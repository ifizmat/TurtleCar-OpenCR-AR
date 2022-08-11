#include <Servo.h>

#define STEERING_PIN         9
#define MOTOR1_START_PIN    60
#define MOTOR1_DIR1_PIN     53
#define MOTOR1_DIR2_PIN     54

#define ANGLE_SET_FORWARD  100
#define ANGLE_TURN_LEFT     65
#define ANGLE_TURN_RIGHT   150

#define TIME_FORWARD      2000
#define TIME_STEER_RIGHT 12500


#define COUNT_ACTIVE_STATES  5

/*
#define STATE_IDLE           0 
#define STATE_MOVE           1
#define STATE_STEER_FORWARD  2
#define STATE_STEER_RIGHT    3
#define STATE_STEER_LEFT     4
#define STATE_MOVE_FORWARD   5
#define STATE_MOVE_RIGHT     6
#define STATE_MOVE_LEFT      7
#define STATE_MOVE_BACK      8
*/

enum {
  STATE_IDLE,
  STATE_MOVE,
  STATE_STEER_FORWARD,
  STATE_STEER_RIGHT,
  STATE_STEER_LEFT,
  STATE_MOVE_FORWARD,
  STATE_MOVE_RIGHT,
  STATE_MOVE_LEFT,
  STATE_MOVE_BACK,
} stateCar;

Servo servoSteering;

int pos = 0;    // variable to store the servo position
int timeNow = 0;
int timeOld = 0;
int numTargetState = 0;
// int stateCar = STATE_IDLE;
int targetState = stateCar;
int listStates[COUNT_ACTIVE_STATES] = {
                                       STATE_IDLE, 
                                       STATE_MOVE_FORWARD, 
                                       STATE_STEER_RIGHT, 
                                       STATE_MOVE_FORWARD,
                                       STATE_IDLE 
                                      };

int listTime[COUNT_ACTIVE_STATES] = {
                                       2000, 
                                       2000, 
                                       12500, 
                                       2000,
                                       2000 
                                      };


enum namesStates {
         state1, 
         state2
       };

struct structState1 {
  namesStates nameState;
  int timeState;
};

struct structState1 stateCar1[2] = { 
                                    state1, 1000,
                                    state2, 2000,
                                   },
             targetState1;

void setup() {
  targetState1.nameState = state2;
  targetState1.timeState = 3000;
  targetState1 = {state1, 3000};
  pinMode(MOTOR1_DIR1_PIN, OUTPUT);
  pinMode(MOTOR1_DIR2_PIN, OUTPUT);
  pinMode(MOTOR1_START_PIN, OUTPUT);

  Serial.begin(115200);
  servoSteering.attach(STEERING_PIN);
  delay(10);

  Serial.println( "STOP: 2000 ms");
  stopMotor();
  delay(2000);

// Forward: pos = SET_FORWARD;
  
  servoSteering.write(ANGLE_SET_FORWARD);
  delay(50);
  timeNow = millis();
  timeOld = timeNow;
  Serial.println( "FIRST: timeNow = " + String(timeNow));
//  targetState = STATE_MOVE_FORWARD;
  targetState = listStates[numTargetState];
}

void loop() {
  timeNow = millis();
  if ((stateCar == listStates[numTargetState]) && (timeNow - timeOld > listTime[numTargetState])) {
    timeOld = timeNow;
    numTargetState++;
    if (numTargetState > COUNT_ACTIVE_STATES - 1) {
      numTargetState = COUNT_ACTIVE_STATES - 1;
    }
    Serial.println( "LOOP: timeNow = " + String(timeNow));
    Serial.println( "numTargetState: " + String(numTargetState));
    Serial.println( "Next Time: " + String(listTime[numTargetState]));
    targetState = listStates[numTargetState];
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


void steerRight() {
    stateCar = STATE_STEER_RIGHT;

    servoSteering.write(ANGLE_TURN_RIGHT);
    delay(50);

    digitalWrite(MOTOR1_START_PIN, HIGH);
    digitalWrite(MOTOR1_DIR1_PIN, LOW);
    digitalWrite(MOTOR1_DIR2_PIN, HIGH);
}
