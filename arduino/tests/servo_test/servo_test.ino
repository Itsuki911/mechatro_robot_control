#include <Servo.h>

const int PIN_SERVO = 10;
const int SERVO_CENTER = 90;
const int SERVO_LEFT_MAX = 58;
const int SERVO_RIGHT_MAX = 122;

Servo steering;

void moveSlowly(int target) {
  int current = steering.read();
  int step = target > current ? 1 : -1;
  while (current != target) {
    current += step;
    steering.write(current);
    delay(25);
  }
}

void setup() {
  steering.attach(PIN_SERVO);
  steering.write(SERVO_CENTER);
}

void loop() {
  moveSlowly(SERVO_LEFT_MAX);
  delay(700);
  moveSlowly(SERVO_CENTER);
  delay(700);
  moveSlowly(SERVO_RIGHT_MAX);
  delay(700);
  moveSlowly(SERVO_CENTER);
  delay(1200);
}
