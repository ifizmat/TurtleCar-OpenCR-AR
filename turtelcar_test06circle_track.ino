#include <Servo.h>

#define STEERING_PIN         9
#define MOTOR1_START_PIN    60
#define MOTOR1_DIR1_PIN     53
#define MOTOR1_DIR2_PIN     54

#define ANGLE_SET_FORWARD  100
#define ANGLE_TURN_LEFT     65
#define ANGLE_TURN_RIGHT   150

// 13.08.2022 removed legacy code: define STATE_, listStates[], listTmes[]

enum possibleStatesCar {
  STATE_IDLE,
  STATE_MOVE,
  STATE_STEER_FORWARD,
  STATE_STEER_RIGHT,
  STATE_STEER_LEFT,
  STATE_MOVE_FORWARD,
  STATE_MOVE_RIGHT,
  STATE_MOVE_LEFT,
  STATE_MOVE_BACK,
} targetState, stateCar;

struct stateChart {
  possibleStatesCar aStateCar;
  int               timeState;
};

#define COUNT_ACTIVE_STATES  8

stateChart transitionsStates[COUNT_ACTIVE_STATES] = {
/*
           STATE_IDLE,          2000, 
           STATE_MOVE_FORWARD,  5000, 
           STATE_STEER_RIGHT,  12800,
           STATE_MOVE_FORWARD,  5000, 
           STATE_STEER_RIGHT,  12800,
*/
           STATE_STEER_FORWARD, 1000,
           STATE_STEER_RIGHT,   2000,
           STATE_STEER_FORWARD, 1000,
           STATE_STEER_RIGHT,   2000,
           STATE_STEER_FORWARD, 1000,
           STATE_STEER_RIGHT,   2000,
           STATE_STEER_FORWARD, 1000,
           STATE_IDLE,          2000                };

Servo servoSteering;

int timeNow = 0;
int timeOld = 0;
int numTargetState = 0;

void setup() {
  pinMode(MOTOR1_DIR1_PIN, OUTPUT);
  pinMode(MOTOR1_DIR2_PIN, OUTPUT);
  pinMode(MOTOR1_START_PIN, OUTPUT);

  Serial.begin(115200);
  servoSteering.attach(STEERING_PIN);
  delay(10);

  Serial.println( "STOP: 2000 ms");
  stopMotor();
  delay(2000);

  servoSteering.write(ANGLE_SET_FORWARD);
  delay(50);
  timeNow = millis();
  timeOld = timeNow;
  Serial.println( "FIRST: timeNow = " + String(timeNow));
//  targetState = STATE_MOVE_FORWARD;
  targetState = transitionsStates[numTargetState].aStateCar;
}

void loop() {
  timeNow = millis();
  if ((stateCar == transitionsStates[numTargetState].aStateCar) && 
      (timeNow - timeOld > transitionsStates[numTargetState].timeState)) {
    timeOld = timeNow;
    numTargetState++;
    if (numTargetState > COUNT_ACTIVE_STATES - 1) {
      numTargetState = COUNT_ACTIVE_STATES - 1;
    }
    targetState = transitionsStates[numTargetState].aStateCar;

    Serial.println( "LOOP: timeNow = " + String(timeNow));
    Serial.println( "numTargetState: " + String(numTargetState));
    Serial.println( "Next Time: " + String(transitionsStates[numTargetState].timeState));
  }

  if (stateCar == targetState) {
    return;
  }

  switch (targetState) {
    case STATE_IDLE:
      stopMotor();
      break;
    case STATE_MOVE_FORWARD:
      runForward();
      break;
    case STATE_STEER_RIGHT:
      steerRight();
      break;
    case STATE_STEER_FORWARD:
      steerForward();
      break;
  }

  delay(50);
}

void stopMotor() {
    stateCar = STATE_IDLE;
    digitalWrite(MOTOR1_START_PIN, LOW);  
}

void runForward() {
    stateCar = STATE_MOVE_FORWARD;

    servoSteering.write(ANGLE_SET_FORWARD);
    delay(50);

    digitalWrite(MOTOR1_START_PIN, HIGH);
    digitalWrite(MOTOR1_DIR1_PIN, LOW);
    digitalWrite(MOTOR1_DIR2_PIN, HIGH);
}

void steerForward() {
    stateCar = STATE_STEER_FORWARD;
    servoSteering.write(ANGLE_SET_FORWARD);
    delay(50);
}

void steerRight() {
    stateCar = STATE_STEER_RIGHT;
    servoSteering.write(ANGLE_TURN_RIGHT);
    delay(50);

    digitalWrite(MOTOR1_START_PIN, HIGH);
    digitalWrite(MOTOR1_DIR1_PIN, LOW);
    digitalWrite(MOTOR1_DIR2_PIN, HIGH);
}
