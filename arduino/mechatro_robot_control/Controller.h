#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Types.h"

void initController();
MotorCommand updateController(const SensorData& sensor);
ControllerTelemetry getControllerTelemetry();
const char* stateName(RobotState state);
bool allLine(const SensorData& sensor);
bool allFloor(const SensorData& sensor);

#endif
