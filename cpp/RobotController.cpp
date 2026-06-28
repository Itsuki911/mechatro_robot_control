/*
  RobotController.cpp

  Arduinoなしで制御ロジックの流れを確認するための簡易実装。
  実機版Controller.cppと同じ白線/黒床、直線/カーブ/ゴールの考え方を使う。
*/

#include "RobotController.h"

#include <algorithm>
#include <cmath>

RobotController::RobotController(const ControllerConfig& config)
    : cfg_(config), state_(RobotState::Init), stateEnteredMs_(0), runStartedMs_(0),
      straightStartedMs_(0), curveStartedMs_(0), goalCandidateMs_(0), lastError_(0) {}

// センサー値を1周期分進め、状態遷移と出力指令を計算する。
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
  // スタート直後はゴール判定を無効化し、横線による誤停止を避ける。
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

  // 中央センサーは直線、外側センサーはカーブ滞在時間の判定に使う。
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

// 現在の状態enumを返す。
RobotState RobotController::state() const {
  return state_;
}

// ログ表示用に状態enumを文字列へ変換する。
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

// 白線/黒床の閾値分類。今回のデモでは白線は高い値、黒床は低い値。
LineColor RobotController::classify(int value) const {
  if (value >= cfg_.whiteLineThreshold) {
    return LineColor::Line;
  }
  if (value <= cfg_.blackFloorThreshold) {
    return LineColor::Floor;
  }
  return LineColor::Unknown;
}

// 状態が変わった時だけ、状態開始時刻を更新する。
void RobotController::setState(RobotState next, unsigned long now) {
  if (state_ != next) {
    state_ = next;
    stateEnteredMs_ = now;
  }
}

// 指定indexのセンサーが白線上かを返す。
bool RobotController::isLine(int index, const SensorData& data) const {
  return index >= 0 && index < 4 && classify(data.color[index]) == LineColor::Line;
}

// 全センサーが白線上ならゴール候補として扱う。
bool RobotController::allLine(const SensorData& data) const {
  return isLine(0, data) && isLine(1, data) && isLine(2, data) && isLine(3, data);
}

// 全センサーが黒床または全ゼロならラインロスト候補として扱う。
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

// S1-S4の白線検出位置から左右誤差を作る。負は左、正は右。
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

// 速度とサーボ角をMotorCommandへ詰める小さなヘルパー。
MotorCommand RobotController::command(float speed, int servoDeg) const {
  MotorCommand cmd;
  cmd.driveSpeed = speed;
  cmd.servoDeg = servoDeg;
  cmd.stop = speed == 0.0f;
  cmd.reverse = speed < 0.0f;
  return cmd;
}

// 停止指令を作る。Calibration、Goal、Emergencyで使う。
MotorCommand RobotController::stopCommand() const {
  return command(0.0f, 90);
}
