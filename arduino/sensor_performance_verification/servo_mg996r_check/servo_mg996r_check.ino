/*
  servo_mg996r_check.ino

  MG996Rサーボを安全寄りの範囲でゆっくり動かし、角度、電源、機械干渉を確認する。
  ロボット本体コードとは独立した、Arduino IDE用の単体性能検証スケッチ。

  配線:
    サーボ信号 -> D10
    サーボ電源 -> 外部電源推奨
    サーボGND と Arduino GND を共通

  シリアルモニター:
    115200bps
*/

#include <Servo.h>

const int SERVO_PIN = 10;
const int SERVO_CENTER = 90;
// まずは機械的にぶつかりにくい狭めの範囲で確認する。
const int SERVO_LEFT_LIMIT = 60;
const int SERVO_RIGHT_LIMIT = 120;
// 角度を急に変えないため、少しずつ動かす。
const int SERVO_STEP_DEG = 2;
const unsigned long STEP_INTERVAL_MS = 120;
const unsigned long HOLD_CENTER_MS = 1000;

Servo steeringServo;
int targetDeg = SERVO_CENTER;
int direction = 1;
unsigned long lastStepMs = 0;
unsigned long centerHoldStartedMs = 0;
bool holdingCenter = true;

void writeServoDeg(int deg) {
  // 誤って範囲外の角度を指定しても、ここで安全範囲に丸める。
  int safeDeg = constrain(deg, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT);
  steeringServo.write(safeDeg);
  Serial.print(millis());
  Serial.print(',');
  Serial.println(safeDeg);
}

void setup() {
  Serial.begin(115200);
  steeringServo.attach(SERVO_PIN);
  delay(500);
  Serial.println(F("time_ms,servo_deg"));
  writeServoDeg(SERVO_CENTER);
  centerHoldStartedMs = millis();
}

void loop() {
  unsigned long now = millis();

  if (holdingCenter) {
    if (now - centerHoldStartedMs >= HOLD_CENTER_MS) {
      holdingCenter = false;
      lastStepMs = now;
      // 中央で待ってから左端へ移動し、左右の往復確認を始める。
      targetDeg = SERVO_LEFT_LIMIT;
      direction = 1;
      writeServoDeg(targetDeg);
    }
    return;
  }

  if (now - lastStepMs < STEP_INTERVAL_MS) {
    return;
  }
  lastStepMs = now;

  targetDeg += direction * SERVO_STEP_DEG;
  if (targetDeg >= SERVO_RIGHT_LIMIT) {
    targetDeg = SERVO_RIGHT_LIMIT;
    // 右端に着いたら左方向へ折り返す。
    direction = -1;
  } else if (targetDeg <= SERVO_LEFT_LIMIT) {
    targetDeg = SERVO_LEFT_LIMIT;
    // 左端に着いたら右方向へ折り返す。
    direction = 1;
  }

  writeServoDeg(targetDeg);
}
