/*
  Sensors.cpp

  実機センサーをArduino APIで読み取り、Controllerが扱いやすいSensorDataへまとめる。
  カラーセンサーはA0-A3へ直結したG出力、距離は超音波TRIG/ECHO、
  姿勢はMPU-6050のI2C読み取り。
  失敗や異常値はerrorFlagsへ入れ、制御側が安全寄りに判断できるようにする。
*/

#include "Sensors.h"

#include <Wire.h>
#include <math.h>

#include "Config.h"

static int previousColor[4] = {0, 0, 0, 0};
static unsigned long lastColorChangeMs = 0;

// 指定したアナログピンを複数回読み、単発ノイズを少し抑えた平均値を返す。
static int readAnalogAverage(int pin) {
  long sum = 0;
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(120);
  }
  return (int)(sum / SENSOR_SAMPLES);
}

// MPU-6050のスリープを解除する。失敗はreadRollDeg側で検出する。
static void initMpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

// MPU-6050の加速度からroll角を概算する。I2C失敗時はok=falseを返す。
static float readRollDeg(bool* ok) {
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
  float ayG = ay / 16384.0;
  float azG = az / 16384.0;
  return atan2(ayG, azG) * 57.2958;
}

// センサー系のピン、I2C、MPUを初期化する。
void initSensors() {
  pinMode(PIN_COLOR_S1, INPUT);
  pinMode(PIN_COLOR_S2, INPUT);
  pinMode(PIN_COLOR_S3, INPUT);
  pinMode(PIN_COLOR_S4, INPUT);
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  Wire.begin();
  initMpu();
  lastColorChangeMs = millis();
}

// 1制御周期分の全センサー値をまとめて読み、異常フラグも同時に作る。
SensorData readSensors(unsigned long nowMs) {
  SensorData data;
  data.timeMs = nowMs;
  data.color[SENSOR_FRONT] = readAnalogAverage(PIN_COLOR_S1);
  data.color[SENSOR_LEFT] = readAnalogAverage(PIN_COLOR_S2);
  data.color[SENSOR_RIGHT] = readAnalogAverage(PIN_COLOR_S3);
  data.color[SENSOR_REAR] = readAnalogAverage(PIN_COLOR_S4);
  data.distanceCm = readUltrasonicDistanceCm(&data.distanceOk);
  data.rollDeg = readRollDeg(&data.mpuOk);
  data.errorFlags = ERROR_NONE;

  // 全ゼロと長時間変化なしは、配線抜けやセンサー停止の候補として扱う。
  data.allZero = true;
  bool changed = false;
  for (int i = 0; i < 4; i++) {
    if (data.color[i] != 0) {
      data.allZero = false;
    }
    if (abs(data.color[i] - previousColor[i]) > SENSOR_STUCK_DELTA) {
      changed = true;
    }
    previousColor[i] = data.color[i];
  }
  if (changed) {
    lastColorChangeMs = nowMs;
  }

  data.colorOk = !data.allZero;
  if (data.allZero) {
    data.errorFlags |= ERROR_COLOR_ALL_ZERO;
  }
  if (!data.distanceOk) {
    data.errorFlags |= ERROR_DISTANCE_TIMEOUT;
  }
  if (!data.mpuOk) {
    data.errorFlags |= ERROR_MPU_READ;
  }
  if (nowMs - lastColorChangeMs >= SENSOR_STUCK_MS) {
    data.errorFlags |= ERROR_SENSOR_STUCK;
  }
  return data;
}

// 白線/黒床の判定。LINE_IS_WHITEで将来の黒線構成にも切り替えられる。
LineColor classifyLineValue(int value) {
  if (LINE_IS_WHITE) {
    if (value >= WHITE_LINE_THRESHOLD) {
      return LINE_DETECTED;
    }
    if (value <= BLACK_FLOOR_THRESHOLD) {
      return LINE_FLOOR;
    }
  } else {
    if (value <= BLACK_FLOOR_THRESHOLD) {
      return LINE_DETECTED;
    }
    if (value >= WHITE_LINE_THRESHOLD) {
      return LINE_FLOOR;
    }
  }
  return LINE_UNKNOWN;
}

// 指定値が「追従すべきライン」と判定されるかを返す。
bool isLineDetectedValue(int value) {
  return classifyLineValue(value) == LINE_DETECTED;
}

// 指定値が床側と判定されるかを返す。ラインロスト判定で使う。
bool isFloorDetectedValue(int value) {
  return classifyLineValue(value) == LINE_FLOOR;
}

// 超音波センサーの距離をcmで返す。pulseInには必ずtimeoutを指定して待ちすぎを防ぐ。
float readUltrasonicDistanceCm(bool* ok) {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    *ok = false;
    return 999.0;
  }
  *ok = true;
  return duration / 58.0;
}
