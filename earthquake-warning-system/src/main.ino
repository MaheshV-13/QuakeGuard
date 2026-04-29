#include <LiquidCrystal.h>
#include <Servo.h>               

#define EARTHQUAKE_BUTTON  2    
#define RESET_BUTTON       3    
#define TEST_BUTTON        4    
#define SENSITIVITY_POT    A0   

#define YELLOW_LED         5    
#define ORANGE_LED         6    
#define RED_LED            7    

#define BUZZER_PIN         8    
#define GAS_SERVO_PIN      9    
#define DOOR_SERVO_PIN     10   

#define COUNTDOWN_DURATION  10     
#define EMERGENCY_DELAY     2000   
#define MINOR_DISPLAY_TIME  3000   
#define DEBOUNCE_DELAY      50     
#define LCD_UPDATE_INTERVAL 500    
#define ANALYSIS_WINDOW     5000   

#define GAS_VALVE_OPEN     0      
#define GAS_VALVE_CLOSED   90     
#define DOOR_LOCKED        0      
#define DOOR_UNLOCKED      90    

#define FREQ_MINOR         1000   
#define FREQ_MODERATE      1500   
#define FREQ_MAJOR         2000   
#define FREQ_SAFE_MODE     500    

enum SystemState {
  STANDBY,           
  TREMOR_DETECTED,   
  ANALYZING,         
  MINOR_QUAKE,       
  MODERATE_QUAKE,    
  MAJOR_QUAKE,       
  EMERGENCY_ACTIVE,  
  SAFE_MODE,         
  SYSTEM_RESETTING   
};

SystemState currentState = STANDBY;
SystemState previousState = STANDBY;

volatile bool earthquakeTriggered = false;
volatile unsigned long lastInterruptTime = 0;

unsigned long tremorStartTime = 0;     
unsigned long stateChangeTime = 0;     
unsigned long lastCountdownUpdate = 0; 
unsigned long lastBeepTime = 0;        
unsigned long lastLCDUpdate = 0;       

bool resetButtonPressed = false;
bool testButtonPressed = false;
bool lastResetState = false;
bool lastTestState = false;
unsigned long lastDebounceTime = 0;

unsigned long tremorDuration = 0;      
int earthquakeSeverity = 0;            
int countdownSeconds = COUNTDOWN_DURATION;
int tremorCount = 0;                   
int tremorPulseCount = 0;              

int emergencyStep = 0;
int resetStep = 0;

bool testMode = false;                 
bool emergencyProtocolsActivated = false;
bool lcdNeedsUpdate = true;            
bool minorBeepPlayed = false;          

int sensitivityValue = 0;              
int lastSensitivityDisplay = -1;       

LiquidCrystal lcd(12, 11, 13, A1, A2, A3);
Servo gasValveServo;
Servo doorLockServo;

void changeState(SystemState newState);
String getStateName(SystemState state);
void readInputs();
void handleStandby();
void handleTremorDetected();
void handleAnalyzing();
void handleMinorQuake();
void handleModerateQuake();
void handleMajorQuake();
void handleEmergencyActive();
void handleSafeMode();
void handleSystemResetting();

void setup() {
  Serial.begin(9600);
  Serial.println("Earthquake Early Warning System v1.0 Initializing...");

  pinMode(EARTHQUAKE_BUTTON, INPUT);
  pinMode(RESET_BUTTON, INPUT);
  pinMode(TEST_BUTTON, INPUT);

  attachInterrupt(digitalPinToInterrupt(EARTHQUAKE_BUTTON), tremorISR, RISING);

  pinMode(YELLOW_LED, OUTPUT);
  pinMode(ORANGE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(RED_LED, LOW);
  noTone(BUZZER_PIN);

  gasValveServo.attach(GAS_SERVO_PIN);
  doorLockServo.attach(DOOR_SERVO_PIN);
  gasValveServo.write(GAS_VALVE_OPEN);
  doorLockServo.write(DOOR_LOCKED);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EQ Warning Sys");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);

  lastBeepTime = millis();
  lastLCDUpdate = millis();

  changeState(STANDBY);
  Serial.println("System Ready!");
}

void loop() {
  readInputs();

  switch (currentState) {
    case STANDBY:
      handleStandby();
      break;
    case TREMOR_DETECTED:
      handleTremorDetected();
      break;
    case ANALYZING:
      handleAnalyzing();
      break;
    case MINOR_QUAKE:
      handleMinorQuake();
      break;
    case MODERATE_QUAKE:
      handleModerateQuake();
      break;
    case MAJOR_QUAKE:
      handleMajorQuake();
      break;
    case EMERGENCY_ACTIVE:
      handleEmergencyActive();
      break;
    case SAFE_MODE:
      handleSafeMode();
      break;
    case SYSTEM_RESETTING:     
      handleSystemResetting();
      break;
    default:
      Serial.println("ERROR: Invalid state! Resetting to STANDBY");
      changeState(STANDBY);
      break;
  }

  if (resetButtonPressed && currentState != STANDBY && currentState != SYSTEM_RESETTING) {
    changeState(SYSTEM_RESETTING); 
    resetButtonPressed = false; 
  }
}

void tremorISR() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 50) { 
    earthquakeTriggered = true;
    lastInterruptTime = interruptTime;
  }
}

void readInputs() {
  unsigned long currentTime = millis();

  bool resetReading = digitalRead(RESET_BUTTON);
  if (resetReading != lastResetState) {
    lastDebounceTime = currentTime;
  }
  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (resetReading == HIGH && !resetButtonPressed) {
      resetButtonPressed = true;
    } else if (resetReading == LOW) {
      resetButtonPressed = false;
    }
  }
  lastResetState = resetReading;

  bool testReading = digitalRead(TEST_BUTTON);
  if (testReading != lastTestState) {
    lastDebounceTime = currentTime;
  }
  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (testReading == HIGH && !testButtonPressed) {
      testButtonPressed = true;
    } else if (testReading == LOW) {
      testButtonPressed = false;
    }
  }
  lastTestState = testReading;

  sensitivityValue = analogRead(SENSITIVITY_POT);
}

void handleStandby() {
  int currentSensitivity = map(sensitivityValue, 0, 1023, 0, 100);
  if (lcdNeedsUpdate || currentSensitivity != lastSensitivityDisplay) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EQ Monitor: SAFE");
    lcd.setCursor(0, 1);
    lcd.print("Sensitivity: ");
    lcd.print(currentSensitivity);
    lcd.print("%  ");
    lastSensitivityDisplay = currentSensitivity;
    lcdNeedsUpdate = false;
    lastLCDUpdate = millis();
  }

  if (earthquakeTriggered) {
    earthquakeTriggered = false; 
    changeState(TREMOR_DETECTED);
    tremorStartTime = millis();
    tremorCount++;
    Serial.print("Earthquake detected! Count: ");
    Serial.println(tremorCount);
  }

  if (testButtonPressed) {
    Serial.println("Test mode activated");
    testMode = true;
    changeState(TREMOR_DETECTED);
    tremorStartTime = millis();
  }
}

void handleTremorDetected() {
  if (lcdNeedsUpdate) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Detecting...");
    lcd.setCursor(0, 1);
    lcd.print("Analyzing Data");
    lcdNeedsUpdate = false;
  }
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(RED_LED, LOW);
  changeState(ANALYZING);
}

void handleAnalyzing() {
  unsigned long currentTime = millis();
  tremorDuration = currentTime - tremorStartTime;

  if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL || lcdNeedsUpdate) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Analyzing...");
    lcd.setCursor(0, 1);
    lcd.print("Time left: ");
    lcd.print((ANALYSIS_WINDOW - tremorDuration) / 1000);
    lcd.print("s  ");
    lastLCDUpdate = currentTime;
    lcdNeedsUpdate = false;
  }

  if (earthquakeTriggered) {
    tremorPulseCount++;
    earthquakeTriggered = false; 
    Serial.print("Vibration pulse counted: ");
    Serial.println(tremorPulseCount);
  }

  if (testMode) {
    tremorPulseCount = 30; 
  }

  if (tremorDuration >= ANALYSIS_WINDOW) {
    Serial.print("Total Pulses Detected: ");
    Serial.println(tremorPulseCount);

    int moderateThreshold = map(sensitivityValue, 0, 1023, 15, 4);
    int majorThreshold = map(sensitivityValue, 0, 1023, 25, 8);

    Serial.print("Dynamic Moderate Threshold: ");
    Serial.println(moderateThreshold);
    Serial.print("Dynamic Major Threshold: ");
    Serial.println(majorThreshold);

    if (tremorPulseCount < moderateThreshold) {
      earthquakeSeverity = 1; 
      changeState(MINOR_QUAKE);
    } else if (tremorPulseCount < majorThreshold) {
      earthquakeSeverity = 2; 
      changeState(MODERATE_QUAKE);
      countdownSeconds = COUNTDOWN_DURATION;
      lastCountdownUpdate = millis();
    } else {
      earthquakeSeverity = 3; 
      changeState(MAJOR_QUAKE);
      countdownSeconds = COUNTDOWN_DURATION;
      lastCountdownUpdate = millis();
    }
    
    Serial.print("Earthquake classified as: ");
    Serial.println(earthquakeSeverity == 1 ? "MINOR" : 
                   earthquakeSeverity == 2 ? "MODERATE" : "MAJOR");
                   
    testMode = false;
    tremorPulseCount = 0;
  }
}

void handleMinorQuake() {
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(RED_LED, LOW);

  if (!minorBeepPlayed) {
    tone(BUZZER_PIN, FREQ_MINOR, 200);  
    minorBeepPlayed = true;
  }

  if (lcdNeedsUpdate) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Minor Tremor    ");
    lcd.setCursor(0, 1);
    lcd.print("Duration: ");
    lcd.print(tremorDuration / 1000);
    lcd.print("s  ");
    lcdNeedsUpdate = false;
  }

  if (millis() - stateChangeTime > MINOR_DISPLAY_TIME) {
    digitalWrite(YELLOW_LED, LOW);
    noTone(BUZZER_PIN);
    minorBeepPlayed = false;  
    changeState(STANDBY);
  }
}

void handleModerateQuake() {
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(ORANGE_LED, HIGH);
  digitalWrite(RED_LED, LOW);

  if (millis() - lastCountdownUpdate >= 1000) {
    countdownSeconds--;
    lastCountdownUpdate = millis();
    lcdNeedsUpdate = true;  
  }

  if (lcdNeedsUpdate) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MODERATE QUAKE! ");
    lcd.setCursor(0, 1);
    lcd.print("Peak in: ");
    lcd.print(countdownSeconds);
    lcd.print("s   ");
    lcdNeedsUpdate = false;
  }

  unsigned long currentTime = millis();
  unsigned long beepCycle = (currentTime - lastBeepTime) % 2000;
  if (beepCycle < 100 || (beepCycle >= 200 && beepCycle < 300) || (beepCycle >= 400 && beepCycle < 500)) {
    tone(BUZZER_PIN, FREQ_MODERATE);
  } else {
    noTone(BUZZER_PIN);
  }

  if (countdownSeconds <= 0) {
    noTone(BUZZER_PIN);
    digitalWrite(ORANGE_LED, LOW);
    changeState(STANDBY);
  }
}

void handleMajorQuake() {
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(RED_LED, HIGH);

  if (millis() - lastCountdownUpdate >= 1000) {
    countdownSeconds--;
    lastCountdownUpdate = millis();
    lcdNeedsUpdate = true;
  }

  if (lcdNeedsUpdate) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MAJOR QUAKE!!!  ");
    lcd.setCursor(0, 1);
    lcd.print("Peak in: ");
    lcd.print(countdownSeconds);
    lcd.print("s   ");
    lcdNeedsUpdate = false;
  }

  tone(BUZZER_PIN, FREQ_MAJOR);

  if (countdownSeconds <= 0) {
    changeState(EMERGENCY_ACTIVE);
  }
}

void handleEmergencyActive() {
  digitalWrite(RED_LED, HIGH);
  unsigned long timeInState = millis() - stateChangeTime;

  if (emergencyStep == 0) {
    Serial.println("Closing gas valve...");
    gasValveServo.write(GAS_VALVE_CLOSED);
    emergencyStep = 1; 
  } 
  else if (emergencyStep == 1 && timeInState >= 300) {
    Serial.println("Unlocking emergency doors...");
    doorLockServo.write(DOOR_UNLOCKED);
    emergencyStep = 2; 
  }

  if (lcdNeedsUpdate) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gas:OFF Door:OPEN");
    lcd.setCursor(0, 1);
    lcd.print("EMERGENCY MODE ");
    lcdNeedsUpdate = false;
  }

  unsigned long beepCycle = (millis() - lastBeepTime) % 1000;
  if (beepCycle < 500) {
    tone(BUZZER_PIN, FREQ_SAFE_MODE);
  } else {
    noTone(BUZZER_PIN);
  }

  if (timeInState > EMERGENCY_DELAY) {
    changeState(SAFE_MODE);
    emergencyStep = 0; 
  }
}

void handleSystemResetting() {
  unsigned long timeInState = millis() - stateChangeTime;

  if (resetStep == 0) {
    Serial.println("PERFORMING SYSTEM RESET...");
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(ORANGE_LED, LOW);
    digitalWrite(RED_LED, LOW);
    noTone(BUZZER_PIN);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Reset    ");
    lcd.setCursor(0, 1);
    lcd.print("Returning...    ");

    Serial.println("Reopening gas valve...");
    gasValveServo.write(GAS_VALVE_OPEN);
    resetStep = 1; 
  }
  else if (resetStep == 1 && timeInState >= 300) {
    Serial.println("Locking doors...");
    doorLockServo.write(DOOR_LOCKED);
    resetStep = 2; 
  }
  else if (resetStep == 2 && timeInState >= 1500) {
    earthquakeSeverity = 0;
    tremorDuration = 0;
    minorBeepPlayed = false;
    resetStep = 0; 
    
    Serial.println("System reset complete!");
    changeState(STANDBY);
  }
}

void handleSafeMode() {
  digitalWrite(RED_LED, HIGH);
  if (lcdNeedsUpdate) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Press RESET to  ");
    lcd.setCursor(0, 1);
    lcd.print("Resume Monitor");
    lcdNeedsUpdate = false;
  }

  unsigned long currentTime = millis();
  unsigned long beepCycle = (currentTime - lastBeepTime) % 2000;
  if (beepCycle < 300) {
    tone(BUZZER_PIN, FREQ_SAFE_MODE);
  } else {
    noTone(BUZZER_PIN);
  }
}

void changeState(SystemState newState) {
  previousState = currentState;
  currentState = newState;
  stateChangeTime = millis();
  lcdNeedsUpdate = true;  
  lastBeepTime = millis(); 
  
  Serial.print("State Change: ");
  Serial.print(getStateName(previousState));
  Serial.print(" -> ");
  Serial.println(getStateName(currentState));
}

String getStateName(SystemState state) {
  switch (state) {
    case STANDBY: return "STANDBY";
    case TREMOR_DETECTED: return "TREMOR_DETECTED";
    case ANALYZING: return "ANALYZING";
    case MINOR_QUAKE: return "MINOR_QUAKE";
    case MODERATE_QUAKE: return "MODERATE_QUAKE";
    case MAJOR_QUAKE: return "MAJOR_QUAKE";
    case EMERGENCY_ACTIVE: return "EMERGENCY_ACTIVE";
    case SAFE_MODE: return "SAFE_MODE";
    case SYSTEM_RESETTING: return "SYSTEM_RESETTING"; 
    default: return "UNKNOWN";
  }
}
