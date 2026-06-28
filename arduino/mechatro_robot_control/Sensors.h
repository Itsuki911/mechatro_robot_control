#ifndef SENSORS_H
#define SENSORS_H

#include "Types.h"

void initSensors();
SensorData readSensors(unsigned long nowMs);
LineColor classifyLineValue(int value);
bool isLineDetectedValue(int value);
bool isFloorDetectedValue(int value);
float readUltrasonicDistanceCm(bool* ok);

#endif
