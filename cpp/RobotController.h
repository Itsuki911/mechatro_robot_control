/*
  RobotController.h

  Arduino非依存の制御判断クラス。
  実機コードの考え方をPCで追うための軽量版で、センサー入力からMotorCommandを返す。
*/

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "MotorCommand.h"
#include "SensorData.h"

// PCデモ用の状態。Arduino版より少し絞り、学習しやすくしている。
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

// 閾値や速度など、PCデモで変更しやすい設定値。
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

// 白線/黒床のセンサー値から、状態とサーボ/速度指令を決めるクラス。
class RobotController {
public:
  // 設定値を受け取って状態機械を初期化する。
  explicit RobotController(const ControllerConfig& config = ControllerConfig());

  // 1周期分のセンサー値を渡し、次に出す指令を受け取る。
  MotorCommand update(const SensorData& data);

  // 現在状態を取得する。テストやログ表示で使う。
  RobotState state() const;
  const char* stateName() const;

  // センサー生値を白線/黒床/不明へ分類する。
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
