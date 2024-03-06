#include <AFMotor.h>
#include <Servo.h>

#define L_WHEEL 1
#define R_WHEEL 2
#define BOTH_WHEELS 3

AF_DCMotor motorL(1);
AF_DCMotor motorR(3);
Servo servo;

const int MIDDLE_SENS_PIN = 13;
const int RELAY_PIN = 12;
const int TRIG_PIN = 8;
const int ECHO_PIN = 10;
const int DISTANCE_THRESHOLD = 25;

const int SERVO_PIN = 9;
const int SERVO_START_POS = 50;
const int SERVO_LEFT_POS = 100;
const int SERVO_RIGHT_POS = 0;

const int HIGH_SPEED = 1800;
uint8_t motorSpeed;

const int IDLING = -1;
const int DETECTED = 1;
const int MOVING = 2;
const int AVOIDING = 3;
int state = IDLING;

void setup() {
  Serial.begin(9600);

  pinMode(MIDDLE_SENS_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  motorL.setSpeed(500);
  motorR.setSpeed(500);

  motorL.run(RELEASE);
  motorR.run(RELEASE);
  
  servo.attach(SERVO_PIN);
  // servo.write(SERVO_START_POS);
}

void setMotorSpeed(int newSpeed) {
  motorL.setSpeed(newSpeed);
  motorR.setSpeed(newSpeed);
  motorSpeed = newSpeed;
}

void runWater(){
  digitalWrite(RELAY_PIN, HIGH);
}

void motorMove(uint8_t wheels, uint8_t direction) {
  if (wheels == BOTH_WHEELS) {
    motorL.run(direction);
    motorR.run(direction);
  } else if (wheels == L_WHEEL) {
    motorL.run(direction);
  } else if (wheels == R_WHEEL) {
    motorR.run(direction);
  } else {
    return;
  }

  if (direction == BRAKE) {
    setMotorSpeed(0);
    return;
  }

  setMotorSpeed(500);
}

void sendWaves() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
}

int getPulseDistance() {
  // https://circuitdigest.com/microcontroller-projects/arduino-ultrasonic-sensor-based-distance-measurement
  int duration = pulseIn(ECHO_PIN, HIGH);
  return duration / 58.2;
}

void swipeHose() {
  int initialPos = servo.read();
  if (initialPos != SERVO_START_POS) {
    if (initialPos > SERVO_START_POS) {
      for (int i = initialPos; i > SERVO_START_POS; i--) {
        servo.write(i);
        delay(10);
      }
    } else {
      for (int i = initialPos; i <= SERVO_START_POS; i++) {
        servo.write(i);
        delay(10);
      }
    }
  }
  
  delay(10);
  for (int i = servo.read(); i <= SERVO_LEFT_POS; i++) {
    servo.write(i);
    delay(10);
  }
  delay(10);
  for (int i = servo.read(); i > SERVO_RIGHT_POS; i--) {
    servo.write(i);
    delay(10);
  }
  delay(10);
  for (int i = servo.read(); i < SERVO_START_POS; i++) {
    servo.write(i);
    delay(10);
  }
}

bool hasObject(int distance) {
  return distance < DISTANCE_THRESHOLD;
}

void loop(){
  // servo.write(100);
  // delay(1000);
  // servo.write(-100);
  // swipeHose();
      // sendWaves();
      // int distance = getPulseDistance();
      // Serial.println(distance);
      // delay(2000);
  int distance = 0;
  int middleSens = digitalRead(MIDDLE_SENS_PIN);
  switch (state) {
    case IDLING:
      Serial.println("IDLING");
      if (middleSens == LOW) {
        state = DETECTED;
      }
      break;
    case DETECTED:
      Serial.println("MOVING");
      motorMove(BOTH_WHEELS, FORWARD);
      state = MOVING;
      break;
    case MOVING:
      delay(1000);
      motorMove(BOTH_WHEELS, BRAKE);
      delay(1000);
      digitalWrite(RELAY_PIN, HIGH);      
      delay(200);
      digitalWrite(RELAY_PIN, LOW);
      swipeHose();
      delay(1000);
      
      digitalWrite(RELAY_PIN, HIGH); 
      state = IDLING;
      delay(2000);
      break;
    case AVOIDING:
      sendWaves();
      int distance = getPulseDistance();
      motorMove(L_WHEEL, FORWARD);
      motorMove(L_WHEEL, BRAKE);

      if(!hasObject(distance)) { 
        motorMove(R_WHEEL, FORWARD);
        motorMove(R_WHEEL, BRAKE);
        state = MOVING;
      }

  }
}