#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include "Types.h"

void initDebugSerial();
void printCsvLog(const SensorData& sensor, const MotorCommand& command,
                 const ControllerTelemetry& telemetry, float estimatedDistanceCm);
void printStateSummary(const ControllerTelemetry& telemetry);
void handleDebugCommands();

#endif
