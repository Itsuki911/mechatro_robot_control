#include "Actuators.h"
#include "Config.h"
#include "Controller.h"
#include "DebugSerial.h"
#include "Sensors.h"

static unsigned long lastControlMs = 0;
static unsigned long lastDistanceUpdateMs = 0;
static float estimatedDistanceCm = 0.0;

static void updateEstimatedDistance(float driveSpeed, unsigned long nowMs) {
  if (lastDistanceUpdateMs == 0) {
    lastDistanceUpdateMs = nowMs;
    return;
  }
  unsigned long dtMs = nowMs - lastDistanceUpdateMs;
  lastDistanceUpdateMs = nowMs;
  float seconds = dtMs / 1000.0;
  estimatedDistanceCm += abs(driveSpeed) * ESTIMATED_SPEED_CM_PER_SEC_AT_FULL_PWM * seconds;
}

void setup() {
  initDebugSerial();
  initSensors();
  initActuators();
  initController();
  Serial.println(F("# mechatro_robot_control servo_rear_dc start"));
}

void loop() {
  unsigned long nowMs = millis();
  handleDebugCommands();
  if (nowMs - lastControlMs < CONTROL_INTERVAL_MS) {
    return;
  }
  lastControlMs = nowMs;

  SensorData sensor = readSensors(nowMs);
  MotorCommand command = updateController(sensor);
  applyMotorCommand(command);
  updateEstimatedDistance(currentDriveSpeed(), nowMs);

  ControllerTelemetry telemetry = getControllerTelemetry();
  printCsvLog(sensor, command, telemetry, estimatedDistanceCm);
  printStateSummary(telemetry);
}
