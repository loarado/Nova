#include <Arduino.h>
#include <SerialBT.h>
#include <Servo.h> // Include the Servo library

/***** Motor & Fan Pin Definitions *****/
// ----- Motor A -----
int MotorA_Pin1   = 3;    // GP3
int MotorA_Pin2   = 4;    // GP4
int MotorA_Speed  = 5;    // GP5 (PWM)

// ----- Motor B -----
int MotorB_Pin1   = 6;    // GP6
int MotorB_Pin2   = 7;    // GP7
int MotorB_Speed  = 1;    // GP1 (PWM)

// ----- Motor C -----
int MotorC_Pin1   = 9;    // GP9
int MotorC_Pin2   = 10;   // GP10
int MotorC_Speed  = 2;    // GP2 (PWM)

// ----- Servo Motors for Arms -----
int Servo_Arm_Hand_Pin = 22;  // GP22
int Servo_Arm_Wrist_Pin = 26;  // GP26
int Servo_Arm_Elbow_Pin = 27;  // GP27
int Servo_Arm_Base_Pin = 28;  // GP28
int Servo_Arm_Claw_Pin = 15;  // GP15

Servo Servo_Arm_Hand;
Servo Servo_Arm_Wrist;
Servo Servo_Arm_Elbow;
Servo Servo_Arm_Base;
Servo Servo_Arm_Claw;

// ----- FAN CONTROL via Motor Controller Channel -----
int Fan_ENA_Pin = 12;    // GP12
int Fan_IN1_Pin = 13;    // GP13
int Fan_IN2_Pin = 14;    // GP14

int currentMotorSpeed = 180;  // Default Motor Speed
int currentFanSpeed = 180;    // Default Fan Speed

const int MAX_SPEED = 255;  // Maximum PWM value (0-255)

/***** Function Prototypes *****/
void moveForward();
void moveBackward();
void moveRight();
void moveLeft();         
void stopMotors();
void stopMotorC();       
void setMotorSpeed(int speed);
void setFanSpeed(int speed);
void handleCommand(char command);

enum RobotState { STATE_STOP=0, STATE_FORWARD, STATE_BACK, STATE_LEFT, STATE_RIGHT };
volatile RobotState currentState = STATE_STOP;

unsigned long lastCmdTime = 0;
const unsigned long SAFETY_TIMEOUT = 5000; // ms (5s)
unsigned long lastPrint = 0;
const unsigned long PRINT_THROTTLE = 200; // ms




void setup() {
  Serial.begin(115200);  // USB serial for debugging
  SerialBT.begin();    // Bluetooth device name
  SerialBT.setName("NovaBT");

  pinMode(MotorA_Pin1, OUTPUT);
  pinMode(MotorA_Pin2, OUTPUT);
  pinMode(MotorA_Speed, OUTPUT);

  pinMode(MotorB_Pin1, OUTPUT);
  pinMode(MotorB_Pin2, OUTPUT);
  pinMode(MotorB_Speed, OUTPUT);

  pinMode(MotorC_Pin1, OUTPUT);
  pinMode(MotorC_Pin2, OUTPUT);
  pinMode(MotorC_Speed, OUTPUT);

  pinMode(Fan_ENA_Pin, OUTPUT);
  pinMode(Fan_IN1_Pin, OUTPUT);
  pinMode(Fan_IN2_Pin, OUTPUT);

  digitalWrite(Fan_IN1_Pin, HIGH);
  digitalWrite(Fan_IN2_Pin, LOW);

  Servo_Arm_Hand.attach(Servo_Arm_Hand_Pin); // Attach the servo object to the specified pin
  
  Servo_Arm_Wrist.attach(Servo_Arm_Wrist_Pin); // Attach the servo object to the specified pin
  
  Servo_Arm_Elbow.attach(Servo_Arm_Elbow_Pin); // Attach the servo object to the specified pin
  Servo_Arm_Base.attach(Servo_Arm_Base_Pin); // Attach the servo object to the specified pin
  Servo_Arm_Claw.attach(Servo_Arm_Claw_Pin); // Attach the servo object to the specified pin


  Servo_Arm_Hand.write(50); // Set the servo to the current angle

  Servo_Arm_Wrist.write(90); // Set the servo to the current angle

  Servo_Arm_Elbow.write(90); // Set the servo to the current angle

  Servo_Arm_Base.write(90); // Set the servo to the current angle

  Servo_Arm_Claw.write(95); // Set the servo to the current angle

  stopMotors();
  setFanSpeed(0);
}

// --- helper that actually applies motor outputs for the current state
void applyState(RobotState s) {
  // debug: show state (throttled)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 200) {
    Serial.print("APPLY_STATE ");
    Serial.println((int)s);
    lastDebug = millis();
  }

  switch(s) {
    case STATE_FORWARD:
      digitalWrite(MotorA_Pin1, LOW); digitalWrite(MotorA_Pin2, HIGH);
      digitalWrite(MotorB_Pin1, LOW); digitalWrite(MotorB_Pin2, HIGH);
      stopMotorC();
      setMotorSpeed(currentMotorSpeed);
      break;
    case STATE_BACK:
      digitalWrite(MotorB_Pin1, HIGH); digitalWrite(MotorB_Pin2, LOW);
      digitalWrite(MotorA_Pin1, HIGH); digitalWrite(MotorA_Pin2, LOW);
      stopMotorC();
      setMotorSpeed(currentMotorSpeed);
      break;
    case STATE_LEFT:
      digitalWrite(MotorA_Pin1, HIGH); digitalWrite(MotorA_Pin2, LOW);
      digitalWrite(MotorB_Pin1, LOW);  digitalWrite(MotorB_Pin2, HIGH);
      digitalWrite(MotorC_Pin1, HIGH); digitalWrite(MotorC_Pin2, LOW);
      setMotorSpeed(currentMotorSpeed);
      break;
    case STATE_RIGHT:
      digitalWrite(MotorB_Pin1, HIGH); digitalWrite(MotorB_Pin2, LOW);
      digitalWrite(MotorC_Pin1, LOW);  digitalWrite(MotorC_Pin2, HIGH);
      digitalWrite(MotorA_Pin1, LOW);  digitalWrite(MotorA_Pin2, HIGH);
      setMotorSpeed(currentMotorSpeed);
      break;
    case STATE_STOP:
    default:
      stopMotors();
      break;
  }
}


void loop() {
  bool gotAny = false;
  char lastCmd = 0;
  // drain buffer and keep last char only
  while (SerialBT.available()) {
    lastCmd = SerialBT.read();
    gotAny = true;
  }
  if (gotAny) {
    handleCommand(lastCmd);
  }

  // safety: if no command for SAFETY_TIMEOUT ms, go to STOP
  if (millis() - lastCmdTime > SAFETY_TIMEOUT) {
    if (currentState != STATE_STOP) {
      currentState = STATE_STOP;
      // optionally print once on safety stop
      if (millis() - lastPrint > PRINT_THROTTLE) {
        Serial.println("SAFETY STOP");
        lastPrint = millis();
      }
    }
  }

  // Always apply the current state every loop tick
  applyState(currentState);

  // short non-blocking delay so loop is not a busy spin
  delay(10);
}



void moveForward() {
  digitalWrite(MotorA_Pin1, LOW); 
  digitalWrite(MotorA_Pin2, HIGH);

  digitalWrite(MotorB_Pin1, LOW); 
  digitalWrite(MotorB_Pin2, HIGH);

  stopMotorC();  // Stop Motor C while going straight
  setMotorSpeed(currentMotorSpeed);
}

void moveBackward() {
  digitalWrite(MotorB_Pin1, HIGH); 
  digitalWrite(MotorB_Pin2, LOW);

  digitalWrite(MotorA_Pin1, HIGH); 
  digitalWrite(MotorA_Pin2, LOW);

  stopMotorC();  // Stop Motor C while moving straight
  setMotorSpeed(currentMotorSpeed);
}

void moveRight() {

  
  digitalWrite(MotorB_Pin1, HIGH); 
  digitalWrite(MotorB_Pin2, LOW);

  digitalWrite(MotorC_Pin1, LOW); 
  digitalWrite(MotorC_Pin2, HIGH);

  digitalWrite(MotorA_Pin1, LOW); 
  digitalWrite(MotorA_Pin2, HIGH);

  
  setMotorSpeed(currentMotorSpeed);
}

void moveLeft() {
  
  digitalWrite(MotorA_Pin1, HIGH); 
  digitalWrite(MotorA_Pin2, LOW);

  digitalWrite(MotorB_Pin1, LOW); 
  digitalWrite(MotorB_Pin2, HIGH);

  digitalWrite(MotorC_Pin1, HIGH); 
  digitalWrite(MotorC_Pin2, LOW);

  setMotorSpeed(currentMotorSpeed);
}



//FUNCTION: Stop only Motor C
void stopMotorC() {
  analogWrite(MotorC_Speed, 0);
  digitalWrite(MotorC_Pin1, LOW);
  digitalWrite(MotorC_Pin2, LOW);
}

void stopMotors() {
  analogWrite(MotorA_Speed, 0);
  analogWrite(MotorB_Speed, 0);
  analogWrite(MotorC_Speed, 0);
}

void setMotorSpeed(int speed) {
  analogWrite(MotorA_Speed, speed);
  analogWrite(MotorB_Speed, speed);
  analogWrite(MotorC_Speed, speed);
}

void setFanSpeed(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  analogWrite(Fan_ENA_Pin, speed);
}

void handleCommand(char command) {
 if (command == 'w') {
    currentState = STATE_FORWARD;
  } else if (command == 's') {
    currentState = STATE_BACK;
  } else if (command == 'l') {
    currentState = STATE_LEFT;
  } else if (command == 'r') {
    currentState = STATE_RIGHT;
  } else if (command == 'q') {
    currentState = STATE_STOP;
  } else if (command == 'a') {
    setFanSpeed(currentFanSpeed);
  } else if (command == 'd') {
    setFanSpeed(0);
  } else if (command == 't') {
    // CLAW RIGHT
  Servo_Arm_Hand.write(50); // Set the servo to the current angle

  } else if (command == 'g') {
    // CLAW LEFT
  Servo_Arm_Hand.write(180); // Set the servo to the current angle

  } else if (command == 'z') {
    // Arm NORMAL
  Servo_Arm_Hand.write(50); // Set the servo to the current angle

  Servo_Arm_Wrist.write(90); // Set the servo to the current angle

  Servo_Arm_Elbow.write(90); // Set the servo to the current angle
  Servo_Arm_Base.write(90); // Set the servo to the current angle
  } else if (command == 'p') {
    // Arm FORWARD
  Servo_Arm_Wrist.write(30); // Set the servo to the current angle

  Servo_Arm_Elbow.write(180); // Set the servo to the current angle
  } else if (command == 'b') {
    // Arm DOWN
  Servo_Arm_Wrist.write(180); // Set the servo to the current angle

  Servo_Arm_Elbow.write(55); // Set the servo to the current angle
  } else if (command == 'c') {
    // BASE RIGHT
  Servo_Arm_Base.write(30); // Set the servo to the current angle
  } else if (command == 'v') {
    // BASE LEFT
  Servo_Arm_Base.write(150); // Set the servo to the current angle
  } else if (command == 'e') {
    // CLAW OPEN
  Servo_Arm_Claw.write(160); // Set the servo to the current angle
  } else if (command == 'f') {
    // CLAW CLOSE
  Servo_Arm_Claw.write(95); // Set the servo to the current angle
  }
  lastCmdTime = millis();
  // throttle prints so we don't slow the MCU
  if (millis() - lastPrint > PRINT_THROTTLE) {
    Serial.print("CMD:");
    Serial.print(command);
    Serial.print(" state=");
    Serial.println((int)currentState);
    lastPrint = millis();
  }
}