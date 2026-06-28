/*
  Config.h

  実機調整で最初に見る設定ファイル。
  ピン番号、白線/黒床の閾値、速度、時間定数をここへ集約し、制御ロジック側に
  マジックナンバーが散らばらないようにしている。
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 旧メカナム構成との差分を明示するための設定。現在の主実装はサーボ+後輪DC。
#define DRIVE_MODE_SERVO_REAR_DC 1
#define DRIVE_MODE_MECANUM_4W 2

const int DRIVE_MODE = DRIVE_MODE_SERVO_REAR_DC;

// MUX経由でS1-S4のカラーセンサーを読むためのピン。
const int PIN_MUX_SIG = A0;
const int PIN_MUX_S0 = 2;
const int PIN_MUX_S1 = 4;
const int PIN_MUX_S2 = A1;

// 前輪サーボと後輪DCモーターのピン。
const int PIN_SERVO = 10;
const int PIN_DRIVE_PWM = 5;
const int PIN_DRIVE_DIR = 7;

// 超音波センサーはアナログ距離センサーではなくデジタルTRIG/ECHOで読む。
const int PIN_ULTRASONIC_TRIG = 8;
const int PIN_ULTRASONIC_ECHO = 9;

const byte MPU_ADDR = 0x68;

// MUXチャンネル割り当て。左からS1, S2, S3, S4。
const int CH_COLOR_S1 = 0;
const int CH_COLOR_S2 = 1;
const int CH_COLOR_S3 = 2;
const int CH_COLOR_S4 = 3;

// 今回は白線/黒床。実測値に合わせて閾値を調整する。
const bool LINE_IS_WHITE = true;
const int WHITE_LINE_THRESHOLD = 700;
const int BLACK_FLOOR_THRESHOLD = 420;
const int SENSOR_SAMPLES = 4;
const int SENSOR_STUCK_DELTA = 4;
const unsigned long SENSOR_STUCK_MS = 2500;

// 速度とサーボは安全側の低め初期値。実機で少しずつ上げる。
const float BASE_SPEED = 0.30;
const float STRAIGHT_SPEED = 0.40;
const float CURVE_SPEED = 0.22;
const float RECOVERY_SEARCH_SPEED = 0.18;
const float BACKTRACK_SPEED = -0.18;
const float MAX_SPEED_STEP = 0.025;

const int SERVO_CENTER = 90;
const int SERVO_LEFT_MAX = 58;
const int SERVO_RIGHT_MAX = 122;
const int MAX_SERVO_STEP = 3;

// 制御周期と状態判定の時間定数。短くしすぎるとノイズに反応しやすい。
const unsigned long CONTROL_INTERVAL_MS = 20;
const unsigned long CALIBRATION_MS = 2500;
const unsigned long SERIAL_INTERVAL_MS = 100;
const unsigned long STATE_LOG_INTERVAL_MS = 1000;
const unsigned long STRAIGHT_CONFIRM_MS = 450;
const unsigned long CURVE_CONFIRM_MS = 220;
const unsigned long GOAL_CONFIRM_MS = 800;
const unsigned long GOAL_IGNORE_START_MS = 3000;
const unsigned long ALL_FLOOR_CONFIRM_MS = 180;
const unsigned long LOST_LINE_FORWARD_MS = 450;
const unsigned long BACKTRACK_MS = 350;
const unsigned long OBSTACLE_STOP_MS = 350;
const unsigned long OBSTACLE_AVOID_MS = 900;

// 障害物、傾き、推定距離の設定。推定距離はエンコーダなしの概算。
const unsigned long ULTRASONIC_TIMEOUT_US = 25000;
const float OBSTACLE_DISTANCE_CM = 18.0;
const float TILT_RECOVER_DEG = 15.0;
const float TILT_STOP_DEG = 30.0;
const float ESTIMATED_SPEED_CM_PER_SEC_AT_FULL_PWM = 28.0;

#endif
