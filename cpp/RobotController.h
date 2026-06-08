#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "MotorCommand.h"
#include "SensorData.h"

enum class RobotState {
  Init,
  Calibration,
  LineTrace,
  CurveEntry,
  CurveTrace,
  ObstacleDetected,
  ObstacleAvoidance,
  LineRecovery,
  TiltRecovery,
  Goal,
  EmergencyStop
};

struct ControllerConfig {
  int blackThreshold = 430;        // これ未満を黒と判定。実機で必ず調整。
  int whiteThreshold = 720;        // これ以上を白と判定。実機で必ず調整。
  float baseSpeed = 0.28f;         // 初期は低速。慣れてから上げる。
  float curveSpeed = 0.22f;
  float recoverySpeed = 0.18f;
  float obstacleDistanceCm = 18.0f;
  float tiltStopDeg = 30.0f;
  float tiltRecoverDeg = 15.0f;
  unsigned long calibrationMs = 2500;
  unsigned long goalConfirmMs = 700;
  unsigned long obstacleAvoidMs = 900;
};

class RobotController {
public:
  explicit RobotController(const ControllerConfig& config = ControllerConfig());

  MotorCommand update(const SensorData& data);
  RobotState state() const;
  const char* stateName() const;
  LineColor classify(int value) const;
  bool isBlack(int index, const SensorData& data) const;

private:
  ControllerConfig cfg_;
  RobotState state_;
  unsigned long stateEnteredMs_;
  unsigned long goalCandidateMs_;
  int lastError_;

  void setState(RobotState next, unsigned long now);
  MotorCommand stopCommand() const;
  MotorCommand mecanum(float vx, float vy, float omega, int servoDeg) const;
  int lineError(const SensorData& data) const;
  bool allWhite(const SensorData& data) const;
  bool goalPattern(const SensorData& data) const;
  const char* nameOf(RobotState state) const;
};

#endif
