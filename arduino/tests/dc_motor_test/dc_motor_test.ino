const int PIN_DRIVE_PWM = 5;
const int PIN_DRIVE_DIR = 7;

void writeMotor(float value) {
  bool forward = value >= 0.0;
  int pwm = (int)(abs(value) * 255.0);
  digitalWrite(PIN_DRIVE_DIR, forward ? HIGH : LOW);
  analogWrite(PIN_DRIVE_PWM, pwm);
}

void setup() {
  pinMode(PIN_DRIVE_PWM, OUTPUT);
  pinMode(PIN_DRIVE_DIR, OUTPUT);
  writeMotor(0.0);
}

void loop() {
  writeMotor(0.18);
  delay(1000);
  writeMotor(0.0);
  delay(1000);
  writeMotor(-0.15);
  delay(600);
  writeMotor(0.0);
  delay(2000);
}
