/*
  dc_motor_test.ino

  後輪DCモーター単体の安全確認用スケッチ。
  低速前進、停止、短時間後退を行い、回転方向と配線を確認する。
*/

const int PIN_DRIVE_PWM = 5;
const int PIN_DRIVE_DIR = 7;

// -1.0..1.0の値をDIRとPWMへ変換してモーターへ出す。
void writeMotor(float value) {
  bool forward = value >= 0.0;
  int pwm = (int)(abs(value) * 255.0);
  digitalWrite(PIN_DRIVE_DIR, forward ? HIGH : LOW);
  analogWrite(PIN_DRIVE_PWM, pwm);
}

// モータードライバーピンを初期化し、停止状態から始める。
void setup() {
  pinMode(PIN_DRIVE_PWM, OUTPUT);
  pinMode(PIN_DRIVE_DIR, OUTPUT);
  writeMotor(0.0);
}

// 低速前進、停止、低速後退、停止を繰り返す。
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
