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

#define STATE_IDLE           0 
#define STATE_MOVE           1
#define STATE_STEER_FORWARD  2
#define STATE_STEER_RIGHT    3
#define STATE_STEER_LEFT     4
#define STATE_MOVE_FORWARD   5
#define STATE_MOVE_RIGHT     6
#define STATE_MOVE_LEFT      7
#define STATE_MOVE_BACK      8

Servo servoSteering;

int pos = 0;    // variable to store the servo position
int timeNow = 0;
int timeOld = 0;
int stateCar = STATE_IDLE;
int targetState = stateCar;

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

// Forward: pos = SET_FORWARD;
  
  servoSteering.write(ANGLE_SET_FORWARD);
  delay(50);
  timeNow = millis();
  timeOld = timeNow;
  Serial.println( "FIRST: timeNow = " + String(timeNow));
//  targetState = STATE_MOVE_FORWARD;
  targetState = STATE_STEER_RIGHT;
}

void loop() {
  timeNow = millis();
  if ((stateCar == STATE_MOVE_FORWARD) && (timeNow - timeOld > TIME_FORWARD)) {
    timeOld = timeNow;
    Serial.println( "LOOP: timeNow = " + String(timeNow));
    targetState = STATE_IDLE;
  }


  if ((stateCar == STATE_STEER_RIGHT) && (timeNow - timeOld > TIME_STEER_RIGHT)) {
    timeOld = timeNow;
    Serial.println( "LOOP: timeNow = " + String(timeNow));
    targetState = STATE_MOVE_FORWARD;
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
