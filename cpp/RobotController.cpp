#include "RobotController.h"

#include <algorithm>
#include <cmath>

RobotController::RobotController(const ControllerConfig& config)
    : cfg_(config), state_(RobotState::Init), stateEnteredMs_(0), runStartedMs_(0),
      straightStartedMs_(0), curveStartedMs_(0), goalCandidateMs_(0), lastError_(0) {}

MotorCommand RobotController::update(const SensorData& data) {
  if (state_ == RobotState::Init) {
    setState(RobotState::Calibration, data.timeMs);
  }
  if (!data.mpuOk || std::fabs(data.rollDeg) >= 30.0f) {
    setState(RobotState::EmergencyStop, data.timeMs);
  }
  if (state_ == RobotState::EmergencyStop || state_ == RobotState::Goal) {
    return stopCommand();
  }
  if (state_ == RobotState::Calibration) {
    if (data.timeMs - stateEnteredMs_ >= cfg_.calibrationMs) {
      runStartedMs_ = data.timeMs;
      setState(RobotState::LineTrace, data.timeMs);
    }
    return stopCommand();
  }
  if (data.distanceOk && data.distanceCm <= cfg_.obstacleDistanceCm) {
    setState(RobotState::ObstacleDetected, data.timeMs);
    return stopCommand();
  }
  if (runStartedMs_ > 0 && data.timeMs - runStartedMs_ >= cfg_.goalIgnoreStartMs && allLine(data)) {
    if (goalCandidateMs_ == 0) {
      goalCandidateMs_ = data.timeMs;
    }
    if (data.timeMs - goalCandidateMs_ >= cfg_.goalConfirmMs) {
      setState(RobotState::Goal, data.timeMs);
      return stopCommand();
    }
  } else {
    goalCandidateMs_ = 0;
  }
  if (allFloor(data)) {
    setState(RobotState::LineRecovery, data.timeMs);
    return command(cfg_.recoverySpeed, 90);
  }

  const bool center = isLine(1, data) || isLine(2, data);
  const bool curve = isLine(0, data) || isLine(3, data);
  if (center) {
    if (straightStartedMs_ == 0) straightStartedMs_ = data.timeMs;
  } else {
    straightStartedMs_ = 0;
  }
  if (curve) {
    if (curveStartedMs_ == 0) curveStartedMs_ = data.timeMs;
  } else {
    curveStartedMs_ = 0;
  }

  const int error = lineError(data);
  lastError_ = error;
  if (curveStartedMs_ && data.timeMs - curveStartedMs_ >= cfg_.curveConfirmMs) {
    setState(RobotState::CurveTrace, data.timeMs);
    return command(cfg_.curveSpeed, std::max(58, std::min(122, 90 - error * 12)));
  }
  if (straightStartedMs_ && data.timeMs - straightStartedMs_ >= cfg_.straightConfirmMs) {
    setState(RobotState::StraightTrace, data.timeMs);
    return command(cfg_.straightSpeed, std::max(58, std::min(122, 90 - error * 7)));
  }
  setState(RobotState::LineTrace, data.timeMs);
  return command(cfg_.baseSpeed, std::max(58, std::min(122, 90 - error * 9)));
}

RobotState RobotController::state() const {
  return state_;
}

const char* RobotController::stateName() const {
  switch (state_) {
    case RobotState::Init: return "INIT";
    case RobotState::Calibration: return "CALIBRATION";
    case RobotState::LineTrace: return "LINE_TRACE";
    case RobotState::StraightTrace: return "STRAIGHT_TRACE";
    case RobotState::CurveTrace: return "CURVE_TRACE";
    case RobotState::LineRecovery: return "LINE_RECOVERY";
    case RobotState::Backtrack: return "BACKTRACK";
    case RobotState::ObstacleDetected: return "OBSTACLE_DETECTED";
    case RobotState::Goal: return "GOAL";
    case RobotState::EmergencyStop: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

LineColor RobotController::classify(int value) const {
  if (value >= cfg_.whiteLineThreshold) {
    return LineColor::Line;
  }
  if (value <= cfg_.blackFloorThreshold) {
    return LineColor::Floor;
  }
  return LineColor::Unknown;
}

void RobotController::setState(RobotState next, unsigned long now) {
  if (state_ != next) {
    state_ = next;
    stateEnteredMs_ = now;
  }
}

bool RobotController::isLine(int index, const SensorData& data) const {
  return index >= 0 && index < 4 && classify(data.color[index]) == LineColor::Line;
}

bool RobotController::allLine(const SensorData& data) const {
  return isLine(0, data) && isLine(1, data) && isLine(2, data) && isLine(3, data);
}

bool RobotController::allFloor(const SensorData& data) const {
  if (data.allZero) {
    return true;
  }
  for (int i = 0; i < 4; ++i) {
    if (classify(data.color[i]) != LineColor::Floor) {
      return false;
    }
  }
  return true;
}

int RobotController::lineError(const SensorData& data) const {
  const int weights[4] = {-3, -1, 1, 3};
  int sum = 0;
  int count = 0;
  for (int i = 0; i < 4; ++i) {
    if (isLine(i, data)) {
      sum += weights[i];
      count++;
    }
  }
  return count == 0 ? lastError_ : sum / count;
}

MotorCommand RobotController::command(float speed, int servoDeg) const {
  MotorCommand cmd;
  cmd.driveSpeed = speed;
  cmd.servoDeg = servoDeg;
  cmd.stop = speed == 0.0f;
  cmd.reverse = speed < 0.0f;
  return cmd;
}

MotorCommand RobotController::stopCommand() const {
  return command(0.0f, 90);
}
