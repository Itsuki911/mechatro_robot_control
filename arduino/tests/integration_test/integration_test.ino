/*
  integration_test.ino

  サーボと後輪DCモーターを組み合わせた低速確認用スケッチ。
  実走行前にタイヤを浮かせ、操舵と駆動が同時に動いても安全かを見る。
*/

#include <Servo.h>

const int PIN_SERVO = 10;
const int PIN_DRIVE_PWM = 5;
const int PIN_DRIVE_DIR = 7;
Servo steering;

// -1.0..1.0の値をDIRとPWMへ変換してモーターへ出す。
void writeMotor(float value) {
  digitalWrite(PIN_DRIVE_DIR, value >= 0.0 ? HIGH : LOW);
  analogWrite(PIN_DRIVE_PWM, (int)(abs(value) * 255.0));
}

// サーボとモータードライバーを初期化する。
void setup() {
  pinMode(PIN_DRIVE_PWM, OUTPUT);
  pinMode(PIN_DRIVE_DIR, OUTPUT);
  steering.attach(PIN_SERVO);
  steering.write(90);
}

// 低速前進しながら中央、左、右へ操舵して、最後に停止する。
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
