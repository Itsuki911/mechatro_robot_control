#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

// Arduino非依存のセンサー入力構造体。
// 実機側ではanalogReadやI2Cで取得した値をこの形に詰め替える。
struct SensorData {
  int color[4] = {0, 0, 0, 0};  // S1..S4の反射強度。大きいほど白、低いほど黒という仮定。
  float distanceCm = 80.0f;      // 前方距離[cm]
  float rollDeg = 0.0f;          // 左右傾き[deg]
  unsigned long timeMs = 0;      // 制御時刻[ms]
};

enum class LineColor {
  Black,
  White,
  Other
};

#endif
