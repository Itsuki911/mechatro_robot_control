/*
  Types.h

  複数ファイルで共有する型定義をまとめる。
  Arduino本体、センサー、制御、ログが同じ構造体を使うことで、関数の引数を
  増やしすぎず、初学者でもデータの流れを追いやすくする。
*/

#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

enum RobotState {
  STATE_INIT,
  STATE_CALIBRATION,
  STATE_LINE_TRACE,
  STATE_STRAIGHT_TRACE,
  STATE_CURVE_ENTRY,
  STATE_CURVE_TRACE,
  STATE_LINE_RECOVERY,
  STATE_BACKTRACK,
  STATE_OBSTACLE_DETECTED,
  STATE_OBSTACLE_AVOIDANCE,
  STATE_TILT_RECOVERY,
  STATE_SENSOR_TEST,
  STATE_MOTOR_TEST,
  STATE_SERVO_TEST,
  STATE_GOAL,
  STATE_EMERGENCY_STOP
};

// センサー値を白線、黒床、不明のどれとして扱うかを表す。
enum LineColor {
  LINE_FLOOR,
  LINE_DETECTED,
  LINE_UNKNOWN
};

// CSVログへ数値で出す異常フラグ。複数異常はビットORで同時に表せる。
enum ErrorFlag {
  ERROR_NONE = 0,
  ERROR_COLOR_ALL_ZERO = 1 << 0,
  ERROR_DISTANCE_TIMEOUT = 1 << 1,
  ERROR_MPU_READ = 1 << 2,
  ERROR_SENSOR_STUCK = 1 << 3,
  ERROR_SERVO_RANGE = 1 << 4,
  ERROR_MOTOR_RANGE = 1 << 5
};

// 状態遷移の節目をCSVで追うためのイベント番号。
enum EventCode {
  EVENT_NONE = 0,
  EVENT_ALL_FLOOR_DETECTED = 1,
  EVENT_LOST_LINE_FORWARD_STARTED = 2,
  EVENT_BACKTRACK_STARTED = 3,
  EVENT_LINE_RECOVERED = 4,
  EVENT_GOAL_CANDIDATE = 5,
  EVENT_GOAL_CONFIRMED = 6
};

// 1制御周期で読んだ入力値。okフラグは異常時の安全判断に使う。
struct SensorData {
  unsigned long timeMs;
  int color[4];
  float distanceCm;
  float rollDeg;
  bool colorOk;
  bool distanceOk;
  bool mpuOk;
  bool allZero;
  unsigned int errorFlags;
};

// ControllerからActuatorsへ渡す目標指令。実出力値はActuators側で滑らかにする。
struct MotorCommand {
  float driveSpeed;
  int servoDeg;
  bool stop;
  bool reverse;
};

// ログ出力用の内部状態。制御に直接必要ない値もPython解析のために残す。
struct ControllerTelemetry {
  RobotState state;
  int linePosition;
  int lineError;
  unsigned long straightMs;
  unsigned long curveMs;
  unsigned int errorFlags;
  int eventCode;
  unsigned long goalCandidateMs;
  unsigned long goalConfirmedMs;
};

#endif
