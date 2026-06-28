/*
  mpu_test.ino

  MPU-6050単体のroll角確認用スケッチ。
  水平、左傾き、右傾きでroll_degが変化するか確認する。
*/

#include <Wire.h>
#include <math.h>

const byte MPU_ADDR = 0x68;

// MPU-6050の加速度からroll角を概算する。読み取り失敗時はok=false。
float readRollDeg(bool* ok) {
  *ok = true;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    *ok = false;
    return 0.0;
  }
  Wire.requestFrom(MPU_ADDR, (byte)6, (byte)true);
  if (Wire.available() < 6) {
    *ok = false;
    return 0.0;
  }
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  (void)ax;
  return atan2(ay / 16384.0, az / 16384.0) * 57.2958;
}

// I2CとMPUを初期化し、CSVヘッダーを出す。
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("time_ms,roll_deg,ok");
}

// 100msごとにroll角とokフラグをCSVで出力する。
void loop() {
  bool ok = false;
  float roll = readRollDeg(&ok);
  Serial.print(millis());
  Serial.print(',');
  Serial.print(roll, 1);
  Serial.print(',');
  Serial.println(ok ? 1 : 0);
  delay(100);
}
