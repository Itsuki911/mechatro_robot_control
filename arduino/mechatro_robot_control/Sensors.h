/*
  Sensors.h

  センサー読み取りAPIの宣言。
  setup()ではinitSensors()、loop()ではreadSensors()を呼ぶだけでよいようにする。
*/

#ifndef SENSORS_H
#define SENSORS_H

#include "Types.h"

// MUX、超音波センサー、I2C/MPUを初期化する。
void initSensors();

// 現在時刻を受け取り、その周期で使うSensorDataをまとめて返す。
SensorData readSensors(unsigned long nowMs);

// 1つのカラーセンサー値を白線/黒床/不明へ分類する。
LineColor classifyLineValue(int value);

// 分類結果をboolで使いたい場所向けの補助関数。
bool isLineDetectedValue(int value);
bool isFloorDetectedValue(int value);

// 超音波距離をcmで読む。ok=falseならtimeoutまたは読み取り失敗。
float readUltrasonicDistanceCm(bool* ok);

#endif
