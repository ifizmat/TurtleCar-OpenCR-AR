#include <Servo.h>

#define STEERING_PIN    9
#define MOTOR1_START_PIN 60
#define MOTOR1_DIR1_PIN  53
#define MOTOR1_DIR2_PIN  54

#define SET_FORWARD   100
#define TURN_LEFT      65
#define TURN_RIGHT    150
#define TIME_FORWARD 2000

#define STATE_IDLE 0
#define STATE_MOVE 1


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
  pos = SET_FORWARD;
  servoSteering.write(pos);
  delay(500);
  timeNow = millis();
  timeOld = timeNow;
  Serial.println( "FIRST: timeNow = " + String(timeNow));
  targetState = STATE_MOVE;
}

void loop() {
  timeNow = millis();
  if ((stateCar == STATE_MOVE) && (timeNow - timeOld > TIME_FORWARD)) {
    timeOld = timeNow;
    Serial.println( "LOOP: timeNow = " + String(timeNow));
    targetState = STATE_IDLE;
  }

  if (stateCar == targetState) {
    return;
  }

  switch (targetState) {
    case STATE_IDLE:
      stopMotor();
      break;
    case STATE_MOVE:
      runForward();
      break;
  }

  delay(50);
}

void stopMotor() {
    stateCar = STATE_IDLE;
    digitalWrite(MOTOR1_START_PIN, LOW);  
}

void runForward() {
    stateCar = STATE_MOVE;
    digitalWrite(MOTOR1_START_PIN, HIGH);
    digitalWrite(MOTOR1_DIR1_PIN, LOW);
    digitalWrite(MOTOR1_DIR2_PIN, HIGH);
}
