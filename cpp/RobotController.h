#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "MotorCommand.h"
#include "SensorData.h"

enum class RobotState {
  Init,
  Calibration,
  LineTrace,
  StraightTrace,
  CurveTrace,
  LineRecovery,
  Backtrack,
  ObstacleDetected,
  Goal,
  EmergencyStop
};

struct ControllerConfig {
  int whiteLineThreshold = 700;
  int blackFloorThreshold = 420;
  float baseSpeed = 0.30f;
  float straightSpeed = 0.40f;
  float curveSpeed = 0.22f;
  float recoverySpeed = 0.18f;
  float obstacleDistanceCm = 18.0f;
  unsigned long calibrationMs = 2500;
  unsigned long straightConfirmMs = 450;
  unsigned long curveConfirmMs = 220;
  unsigned long goalIgnoreStartMs = 3000;
  unsigned long goalConfirmMs = 800;
};

class RobotController {
public:
  explicit RobotController(const ControllerConfig& config = ControllerConfig());

  MotorCommand update(const SensorData& data);
  RobotState state() const;
  const char* stateName() const;
  LineColor classify(int value) const;

private:
  ControllerConfig cfg_;
  RobotState state_;
  unsigned long stateEnteredMs_;
  unsigned long runStartedMs_;
  unsigned long straightStartedMs_;
  unsigned long curveStartedMs_;
  unsigned long goalCandidateMs_;
  int lastError_;

  void setState(RobotState next, unsigned long now);
  bool isLine(int index, const SensorData& data) const;
  bool allLine(const SensorData& data) const;
  bool allFloor(const SensorData& data) const;
  int lineError(const SensorData& data) const;
  MotorCommand command(float speed, int servoDeg) const;
  MotorCommand stopCommand() const;
};

#endif
