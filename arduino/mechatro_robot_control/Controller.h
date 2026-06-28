/*
  Controller.h

  状態遷移と目標指令を決めるControllerの公開API。
  Arduinoのloop()からは、センサー値を渡してMotorCommandを受け取るだけにする。
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Types.h"

// 状態機械を初期化する。setup()で1回呼ぶ。
void initController();

// センサー値から、次周期の目標サーボ角と駆動速度を返す。
MotorCommand updateController(const SensorData& sensor);

// CSVログ用に、直近の状態・誤差・滞在時間を取得する。
ControllerTelemetry getControllerTelemetry();

// RobotStateをログ用文字列に変換する。
const char* stateName(RobotState state);

// ゴール判定やラインロスト判定で使う共通関数。
bool allLine(const SensorData& sensor);
bool allFloor(const SensorData& sensor);

#endif
