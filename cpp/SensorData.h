/*
  SensorData.h

  PC上で制御判断を試すためのセンサー入力型。
  Arduino APIに依存しないので、g++だけで状態遷移の流れを確認できる。
*/

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

// Arduino版のSensorDataと同じ意味を持つ、C++デモ用の入力構造体。
struct SensorData {
  int color[4] = {0, 0, 0, 0};
  float distanceCm = 999.0f;
  float rollDeg = 0.0f;
  unsigned long timeMs = 0;
  bool colorOk = true;
  bool distanceOk = true;
  bool mpuOk = true;
  bool allZero = false;
};

// 白線/黒床/不明を表す分類結果。
enum class LineColor {
  Floor,
  Line,
  Unknown
};

#endif
