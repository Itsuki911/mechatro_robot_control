#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

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

enum class LineColor {
  Floor,
  Line,
  Unknown
};

#endif
