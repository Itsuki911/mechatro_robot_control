/*
  DebugSerial.h

  シリアルログ出力APIの宣言。
  Pythonで扱えるCSV出力と、シリアルモニター用の短い状態表示を分けている。
*/

#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include "Types.h"

// 115200bpsで開始し、CSVヘッダーを出す。
void initDebugSerial();

// 1周期分のログをCSV形式で出す。
void printCsvLog(const SensorData& sensor, const MotorCommand& command,
                 const ControllerTelemetry& telemetry, float estimatedDistanceCm);

// 1秒ごとの人間向け状態サマリーを出す。
void printStateSummary(const ControllerTelemetry& telemetry);

// シリアル入力から簡単な停止/ヘルプコマンドを処理する。
void handleDebugCommands();

#endif
