/*
  Actuators.h

  サーボと後輪DCモーター出力APIの宣言。
  Controllerの目標値を受け取り、Actuators.cppで安全な変化量に制限してから出力する。
*/

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "Types.h"

// 出力ピンとServoを初期化し、停止状態へ入る。
void initActuators();

// 目標指令を実ピンへ反映する。速度/角度のランプ処理もここで行う。
void applyMotorCommand(const MotorCommand& command);

// 即時停止し、サーボを中央へ戻す。
void stopActuators();

// CSVログ用の現在出力値。
float currentDriveSpeed();
int currentServoDeg();

// サーボ角やモーター指令の範囲外エラーを取得する。
unsigned int actuatorErrorFlags();

#endif
