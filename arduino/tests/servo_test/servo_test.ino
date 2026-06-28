/*
  servo_test.ino

  サーボ単体の安全確認用スケッチ。
  中央、左、右へゆっくり動かし、機械的にぶつからない角度を確認する。
*/

#include <Servo.h>

const int PIN_SERVO = 10;
const int SERVO_CENTER = 90;
const int SERVO_LEFT_MAX = 58;
const int SERVO_RIGHT_MAX = 122;

Servo steering;

// 急に角度を変えず、1度ずつゆっくり目標角へ動かす。
void moveSlowly(int target) {
  int current = steering.read();
  int step = target > current ? 1 : -1;
  while (current != target) {
    current += step;
    steering.write(current);
    delay(25);
  }
}

// サーボを接続し、最初は中央へ置く。
void setup() {
  steering.attach(PIN_SERVO);
  steering.write(SERVO_CENTER);
}

// 左、中央、右、中央の順で繰り返し動かす。
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
