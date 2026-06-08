#include "RobotController.h"

#include <algorithm>
#include <cmath>

RobotController::RobotController(const ControllerConfig& config)
    : cfg_(config), state_(RobotState::Init), stateEnteredMs_(0),
      goalCandidateMs_(0), lastError_(0) {}

MotorCommand RobotController::update(const SensorData& data) {
  if (state_ == RobotState::Init) {
    setState(RobotState::Calibration, data.timeMs);
  }

  if (std::fabs(data.rollDeg) >= cfg_.tiltStopDeg) {
    setState(RobotState::EmergencyStop, data.timeMs);
  }

  if (state_ == RobotState::EmergencyStop) {
    return stopCommand();
  }

  if (state_ == RobotState::Calibration) {
    if (data.timeMs - stateEnteredMs_ >= cfg_.calibrationMs) {
      setState(RobotState::LineTrace, data.timeMs);
    }
    return stopCommand();
  }

  if (std::fabs(data.rollDeg) >= cfg_.tiltRecoverDeg) {
    setState(RobotState::TiltRecovery, data.timeMs);
  } else if (data.distanceCm > 0.0f && data.distanceCm <= cfg_.obstacleDistanceCm) {
    setState(RobotState::ObstacleDetected, data.timeMs);
  }

  if (goalPattern(data)) {
    if (goalCandidateMs_ == 0) {
      goalCandidateMs_ = data.timeMs;
    }
    if (data.timeMs - goalCandidateMs_ >= cfg_.goalConfirmMs) {
      setState(RobotState::Goal, data.timeMs);
    }
  } else {
    goalCandidateMs_ = 0;
  }

  switch (state_) {
    case RobotState::TiltRecovery:
      if (std::fabs(data.rollDeg) < cfg_.tiltRecoverDeg * 0.6f) {
        setState(RobotState::LineTrace, data.timeMs);
      }
      return mecanum(cfg_.recoverySpeed * 0.5f, 0.0f,
                     data.rollDeg > 0.0f ? -0.18f : 0.18f, 90);

    case RobotState::ObstacleDetected:
      setState(RobotState::ObstacleAvoidance, data.timeMs);
      return stopCommand();

    case RobotState::ObstacleAvoidance:
      if (data.timeMs - stateEnteredMs_ >= cfg_.obstacleAvoidMs) {
        setState(RobotState::LineRecovery, data.timeMs);
      }
      // メカナムなら右へ横移動しながら微前進。2輪構成ではこの指令は旋回回避に置換する。
      return mecanum(cfg_.recoverySpeed, -0.18f, 0.10f, 115);

    case RobotState::LineRecovery:
      if (!allWhite(data)) {
        setState(RobotState::LineTrace, data.timeMs);
      }
      return mecanum(cfg_.recoverySpeed, 0.0f, lastError_ >= 0 ? -0.22f : 0.22f,
                     lastError_ >= 0 ? 70 : 110);

    case RobotState::CurveEntry:
    case RobotState::CurveTrace:
    case RobotState::LineTrace: {
      if (allWhite(data)) {
        setState(RobotState::LineRecovery, data.timeMs);
        return mecanum(0.0f, 0.0f, lastError_ >= 0 ? -0.18f : 0.18f, 90);
      }

      const bool s1 = isBlack(0, data);
      const bool s2 = isBlack(1, data);
      const bool s3 = isBlack(2, data);
      const bool s4 = isBlack(3, data);

      if ((s1 || s4) && !(s2 && s3)) {
        setState(RobotState::CurveEntry, data.timeMs);
      }
      if ((s1 && s4) && !s2 && !s3) {
        setState(RobotState::LineTrace, data.timeMs);
      } else if (state_ == RobotState::CurveEntry) {
        setState(RobotState::CurveTrace, data.timeMs);
      }

      const int error = lineError(data);
      lastError_ = error;
      const float gain = state_ == RobotState::CurveTrace ? 0.085f : 0.055f;
      const float speed = state_ == RobotState::CurveTrace ? cfg_.curveSpeed : cfg_.baseSpeed;
      const float omega = std::max(-0.45f, std::min(0.45f, -gain * static_cast<float>(error)));
      const int servo = std::max(55, std::min(125, 90 - error * 7));
      return mecanum(speed, 0.0f, omega, servo);
    }

    case RobotState::Goal:
      return stopCommand();

    default:
      return stopCommand();
  }
}

RobotState RobotController::state() const {
  return state_;
}

const char* RobotController::stateName() const {
  return nameOf(state_);
}

LineColor RobotController::classify(int value) const {
  if (value <= cfg_.blackThreshold) {
    return LineColor::Black;
  }
  if (value >= cfg_.whiteThreshold) {
    return LineColor::White;
  }
  return LineColor::Other;
}

bool RobotController::isBlack(int index, const SensorData& data) const {
  return index >= 0 && index < 4 && classify(data.color[index]) == LineColor::Black;
}

void RobotController::setState(RobotState next, unsigned long now) {
  if (state_ != next) {
    state_ = next;
    stateEnteredMs_ = now;
  }
}

MotorCommand RobotController::stopCommand() const {
  MotorCommand cmd;
  cmd.stop = true;
  cmd.servoDeg = 90;
  return cmd;
}

MotorCommand RobotController::mecanum(float vx, float vy, float omega, int servoDeg) const {
  MotorCommand cmd;
  cmd.frontLeft = vx + vy + omega;
  cmd.frontRight = vx - vy - omega;
  cmd.rearLeft = vx - vy + omega;
  cmd.rearRight = vx + vy - omega;
  const float maxAbs = std::max({1.0f, std::fabs(cmd.frontLeft), std::fabs(cmd.frontRight),
                                std::fabs(cmd.rearLeft), std::fabs(cmd.rearRight)});
  cmd.frontLeft /= maxAbs;
  cmd.frontRight /= maxAbs;
  cmd.rearLeft /= maxAbs;
  cmd.rearRight /= maxAbs;
  cmd.servoDeg = servoDeg;
  cmd.stop = false;
  return cmd;
}

int RobotController::lineError(const SensorData& data) const {
  // S1,S2,S3,S4を左から右へ並べた仮定。黒がある位置の重み平均で補正量を作る。
  const int weights[4] = {-3, -1, 1, 3};
  int sum = 0;
  int count = 0;
  for (int i = 0; i < 4; ++i) {
    if (isBlack(i, data)) {
      sum += weights[i];
      count++;
    }
  }
  if (count == 0) {
    return lastError_;
  }
  return sum / count;
}

bool RobotController::allWhite(const SensorData& data) const {
  for (int i = 0; i < 4; ++i) {
    if (classify(data.color[i]) != LineColor::White) {
      return false;
    }
  }
  return true;
}

bool RobotController::goalPattern(const SensorData& data) const {
  // ゴールはスタート付近の横棒を一定時間検出する想定。単発交差線で止まらないよう時間確認する。
  return isBlack(0, data) && isBlack(1, data) && isBlack(2, data) && isBlack(3, data);
}

const char* RobotController::nameOf(RobotState state) const {
  switch (state) {
    case RobotState::Init: return "Init";
    case RobotState::Calibration: return "Calibration";
    case RobotState::LineTrace: return "LineTrace";
    case RobotState::CurveEntry: return "CurveEntry";
    case RobotState::CurveTrace: return "CurveTrace";
    case RobotState::ObstacleDetected: return "ObstacleDetected";
    case RobotState::ObstacleAvoidance: return "ObstacleAvoidance";
    case RobotState::LineRecovery: return "LineRecovery";
    case RobotState::TiltRecovery: return "TiltRecovery";
    case RobotState::Goal: return "Goal";
    case RobotState::EmergencyStop: return "EmergencyStop";
  }
  return "Unknown";
}
