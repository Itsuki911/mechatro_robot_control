#include <Servo.h>

const int PIN_SERVO = 10;
const int PIN_DRIVE_PWM = 5;
const int PIN_DRIVE_DIR = 7;
Servo steering;

void writeMotor(float value) {
  digitalWrite(PIN_DRIVE_DIR, value >= 0.0 ? HIGH : LOW);
  analogWrite(PIN_DRIVE_PWM, (int)(abs(value) * 255.0));
}

void setup() {
  pinMode(PIN_DRIVE_PWM, OUTPUT);
  pinMode(PIN_DRIVE_DIR, OUTPUT);
  steering.attach(PIN_SERVO);
  steering.write(90);
}

void loop() {
  steering.write(90);
  writeMotor(0.16);
  delay(800);
  steering.write(70);
  delay(600);
  steering.write(110);
  delay(600);
  writeMotor(0.0);
  steering.write(90);
  delay(1500);
}
