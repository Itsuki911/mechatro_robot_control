/*
  Controller.cpp

  ロボットの状態遷移と、次に出すサーボ角/駆動速度を決める中心ファイル。
  実機出力はActuators.cpp、センサー読み取りはSensors.cppへ分け、ここでは
  「白線をどう追うか」「危険時にどう止めるか」という判断だけを扱う。
*/

#include "Controller.h"

#include "Config.h"
#include "Sensors.h"

static RobotState currentState = STATE_INIT;
static unsigned long stateEnteredMs = 0;
static unsigned long runStartedMs = 0;
static unsigned long straightStartedMs = 0;
static unsigned long curveStartedMs = 0;
static unsigned long allFloorStartedMs = 0;
static unsigned long recoveryStartedMs = 0;
static unsigned long backtrackStartedMs = 0;
static unsigned long goalCandidateStartedMs = 0;
static int lastLineError = 0;
static ControllerTelemetry telemetry;

// 状態が変わった時だけ、状態名と入った時刻を更新する。
static void setState(RobotState next, unsigned long nowMs) {
  if (currentState != next) {
    currentState = next;
    stateEnteredMs = nowMs;
  }
}

// S1-S4のindexに対応するセンサーが白線を検出しているかを返す。
// ひし形配置: S1=前方, S2=左, S3=右, S4=後方。
static bool lineAt(const SensorData& sensor, int index) {
  return isLineDetectedValue(sensor.color[index]);
}

// 黒床または全ゼロを床側として扱う。全ゼロは配線異常でもラインロストでも安全側に倒す。
static bool floorAt(const SensorData& sensor, int index) {
  return sensor.allZero || isFloorDetectedValue(sensor.color[index]);
}

// ゴール候補など、4センサーすべてが白線上にいるかを判定する。
bool allLine(const SensorData& sensor) {
  return lineAt(sensor, 0) && lineAt(sensor, 1) && lineAt(sensor, 2) && lineAt(sensor, 3);
}

// ラインロスト候補として、4センサーすべてが床側かを判定する。
bool allFloor(const SensorData& sensor) {
  return floorAt(sensor, 0) && floorAt(sensor, 1) && floorAt(sensor, 2) && floorAt(sensor, 3);
}

// 白線位置の左右誤差を計算する。負は左、正は右、0は中央。
// ひし形配置では前方S1と後方S4は中央軸、S2/S3を左右補正に使う。
static int calcLineError(const SensorData& sensor) {
  const int weights[4] = {0, -2, 2, 0};
  int sum = 0;
  int count = 0;
  for (int i = 0; i < 4; i++) {
    if (lineAt(sensor, i)) {
      sum += weights[i];
      count++;
    }
  }
  if (count == 0) {
    return lastLineError;
  }
  return sum / count;
}

// CSV表示用のライン位置。誤差より少し細かいスケールで見たい時に使う。
static int calcLinePosition(const SensorData& sensor) {
  const int positions[4] = {0, -30, 30, 0};
  int sum = 0;
  int count = 0;
  for (int i = 0; i < 4; i++) {
    if (lineAt(sensor, i)) {
      sum += positions[i];
      count++;
    }
  }
  if (count == 0) {
    return lastLineError * 10;
  }
  return sum / count;
}

// 停止指令を作る。Goal/Emergency/Calibrationで共通利用する。
static MotorCommand stopCommand() {
  MotorCommand cmd;
  cmd.driveSpeed = 0.0;
  cmd.servoDeg = SERVO_CENTER;
  cmd.stop = true;
  cmd.reverse = false;
  return cmd;
}

// サーボ角と前後速度からMotorCommandを作る。負速度は短時間バック用。
static MotorCommand driveCommand(float speed, int servoDeg) {
  MotorCommand cmd;
  cmd.driveSpeed = speed;
  cmd.servoDeg = servoDeg;
  cmd.stop = speed == 0.0;
  cmd.reverse = speed < 0.0;
  return cmd;
}

// 中央軸センサー/左右センサーの白線滞在時間を更新し、直線/カーブ判定に使う。
// 直線: 前方S1または後方S4が白線上にいる時間。
// カーブ: 左S2または右S3が白線上にいる時間。
static void updateLineTimers(const SensorData& sensor, unsigned long nowMs) {
  bool centerLine = lineAt(sensor, SENSOR_FRONT) || lineAt(sensor, SENSOR_REAR);
  bool leftCurveLine = lineAt(sensor, SENSOR_LEFT);
  bool rightCurveLine = lineAt(sensor, SENSOR_RIGHT);

  if (centerLine) {
    if (straightStartedMs == 0) {
      straightStartedMs = nowMs;
    }
  } else {
    straightStartedMs = 0;
  }

  if (leftCurveLine || rightCurveLine) {
    if (curveStartedMs == 0) {
      curveStartedMs = nowMs;
    }
  } else {
    curveStartedMs = 0;
  }

  telemetry.straightMs = straightStartedMs == 0 ? 0 : nowMs - straightStartedMs;
  telemetry.curveMs = curveStartedMs == 0 ? 0 : nowMs - curveStartedMs;
}

// 起動時またはテスト再開始時に、状態機械の内部記録を初期値へ戻す。
void initController() {
  currentState = STATE_INIT;
  stateEnteredMs = millis();
  runStartedMs = 0;
  straightStartedMs = 0;
  curveStartedMs = 0;
  allFloorStartedMs = 0;
  recoveryStartedMs = 0;
  backtrackStartedMs = 0;
  goalCandidateStartedMs = 0;
  lastLineError = 0;
}

// センサー値から次周期の目標速度と目標サーボ角を決めるメイン関数。
MotorCommand updateController(const SensorData& sensor) {
  unsigned long nowMs = sensor.timeMs;
  telemetry.eventCode = EVENT_NONE;
  telemetry.errorFlags = sensor.errorFlags;
  updateLineTimers(sensor, nowMs);

  if (currentState == STATE_INIT) {
    setState(STATE_CALIBRATION, nowMs);
  }

  if ((sensor.errorFlags & ERROR_COLOR_ALL_ZERO) != 0) {
    telemetry.eventCode = EVENT_ALL_FLOOR_DETECTED;
  }

  if (abs(sensor.rollDeg) >= TILT_STOP_DEG) {
    setState(STATE_EMERGENCY_STOP, nowMs);
  }

  if (currentState == STATE_EMERGENCY_STOP || currentState == STATE_GOAL) {
    telemetry.state = currentState;
    return stopCommand();
  }

  if (currentState == STATE_CALIBRATION) {
    if (nowMs - stateEnteredMs >= CALIBRATION_MS) {
      runStartedMs = nowMs;
      setState(STATE_LINE_TRACE, nowMs);
    }
    telemetry.state = currentState;
    return stopCommand();
  }

  // 危険度が高い条件はライン追従より優先して処理する。
  if (!sensor.mpuOk || abs(sensor.rollDeg) >= TILT_RECOVER_DEG) {
    setState(STATE_TILT_RECOVERY, nowMs);
  } else if (sensor.distanceOk && sensor.distanceCm <= OBSTACLE_DISTANCE_CM) {
    setState(STATE_OBSTACLE_DETECTED, nowMs);
  }

  // スタート直後の横線で止まらないよう、一定時間走ってからゴール判定する。
  if (runStartedMs > 0 && nowMs - runStartedMs >= GOAL_IGNORE_START_MS && allLine(sensor)) {
    if (goalCandidateStartedMs == 0) {
      goalCandidateStartedMs = nowMs;
      telemetry.goalCandidateMs = nowMs;
      telemetry.eventCode = EVENT_GOAL_CANDIDATE;
    }
    if (nowMs - goalCandidateStartedMs >= GOAL_CONFIRM_MS) {
      telemetry.goalConfirmedMs = nowMs;
      telemetry.eventCode = EVENT_GOAL_CONFIRMED;
      setState(STATE_GOAL, nowMs);
      telemetry.state = currentState;
      return stopCommand();
    }
  } else {
    goalCandidateStartedMs = 0;
  }

  if (currentState == STATE_TILT_RECOVERY) {
    if (sensor.mpuOk && abs(sensor.rollDeg) < TILT_RECOVER_DEG * 0.6) {
      setState(STATE_LINE_TRACE, nowMs);
    }
    int servo = sensor.rollDeg > 0.0 ? SERVO_LEFT_MAX + 10 : SERVO_RIGHT_MAX - 10;
    telemetry.state = currentState;
    return driveCommand(RECOVERY_SEARCH_SPEED * 0.5, servo);
  }

  if (currentState == STATE_OBSTACLE_DETECTED) {
    if (nowMs - stateEnteredMs >= OBSTACLE_STOP_MS) {
      setState(STATE_OBSTACLE_AVOIDANCE, nowMs);
    }
    telemetry.state = currentState;
    return stopCommand();
  }

  if (currentState == STATE_OBSTACLE_AVOIDANCE) {
    if (nowMs - stateEnteredMs >= OBSTACLE_AVOID_MS) {
      setState(STATE_LINE_RECOVERY, nowMs);
      recoveryStartedMs = nowMs;
    }
    telemetry.state = currentState;
    return driveCommand(RECOVERY_SEARCH_SPEED, SERVO_RIGHT_MAX - 8);
  }

  // 全センサー床側なら、すぐ停止せず低速直進から復帰を試す。
  if (allFloor(sensor)) {
    if (allFloorStartedMs == 0) {
      allFloorStartedMs = nowMs;
    }
    if (nowMs - allFloorStartedMs >= ALL_FLOOR_CONFIRM_MS &&
        currentState != STATE_LINE_RECOVERY && currentState != STATE_BACKTRACK) {
      setState(STATE_LINE_RECOVERY, nowMs);
      recoveryStartedMs = nowMs;
      telemetry.eventCode = EVENT_LOST_LINE_FORWARD_STARTED;
    }
  } else {
    allFloorStartedMs = 0;
  }

  // 復帰中は、まず直進して白線再検出を待ち、それでもだめなら短時間バックする。
  if (currentState == STATE_LINE_RECOVERY) {
    if (!allFloor(sensor) && (lineAt(sensor, 0) || lineAt(sensor, 1) || lineAt(sensor, 2) || lineAt(sensor, 3))) {
      setState(STATE_LINE_TRACE, nowMs);
      telemetry.eventCode = EVENT_LINE_RECOVERED;
    } else if (nowMs - recoveryStartedMs < LOST_LINE_FORWARD_MS) {
      telemetry.state = currentState;
      return driveCommand(RECOVERY_SEARCH_SPEED, SERVO_CENTER);
    } else {
      setState(STATE_BACKTRACK, nowMs);
      backtrackStartedMs = nowMs;
      telemetry.eventCode = EVENT_BACKTRACK_STARTED;
    }
  }

  // バック後も復帰できない場合は、再び低速探索へ戻す。
  if (currentState == STATE_BACKTRACK) {
    if (!allFloor(sensor) && (lineAt(sensor, 0) || lineAt(sensor, 1) || lineAt(sensor, 2) || lineAt(sensor, 3))) {
      setState(STATE_LINE_TRACE, nowMs);
      telemetry.eventCode = EVENT_LINE_RECOVERED;
    } else if (nowMs - backtrackStartedMs < BACKTRACK_MS) {
      int servo = lastLineError >= 0 ? SERVO_LEFT_MAX + 8 : SERVO_RIGHT_MAX - 8;
      telemetry.state = currentState;
      return driveCommand(BACKTRACK_SPEED, servo);
    } else {
      recoveryStartedMs = nowMs;
      setState(STATE_LINE_RECOVERY, nowMs);
    }
  }

  int error = calcLineError(sensor);
  lastLineError = error;
  int linePos = calcLinePosition(sensor);

  // 滞在時間で状態を決めることで、一瞬のノイズで直線/カーブ判定しない。
  if (telemetry.curveMs >= CURVE_CONFIRM_MS) {
    setState(currentState == STATE_CURVE_TRACE ? STATE_CURVE_TRACE : STATE_CURVE_ENTRY, nowMs);
    if (currentState == STATE_CURVE_ENTRY) {
      setState(STATE_CURVE_TRACE, nowMs);
    }
  } else if (telemetry.straightMs >= STRAIGHT_CONFIRM_MS) {
    setState(STATE_STRAIGHT_TRACE, nowMs);
  } else if (currentState != STATE_CURVE_TRACE) {
    setState(STATE_LINE_TRACE, nowMs);
  }

  float speed = BASE_SPEED;
  if (currentState == STATE_STRAIGHT_TRACE) {
    speed = STRAIGHT_SPEED;
  } else if (currentState == STATE_CURVE_TRACE || currentState == STATE_CURVE_ENTRY) {
    speed = CURVE_SPEED;
  }

  int servo = SERVO_CENTER - error * 9;
  if (currentState == STATE_CURVE_TRACE) {
    servo = SERVO_CENTER - error * 12;
  }
  servo = constrain(servo, SERVO_LEFT_MAX, SERVO_RIGHT_MAX);

  telemetry.state = currentState;
  telemetry.linePosition = linePos;
  telemetry.lineError = error;
  telemetry.errorFlags = sensor.errorFlags;
  return driveCommand(speed, servo);
}

// DebugSerial.cppがCSVに出すため、直近の状態・誤差・滞在時間を返す。
ControllerTelemetry getControllerTelemetry() {
  telemetry.state = currentState;
  return telemetry;
}

// enumをCSVで読みやすい短い文字列へ変換する。
const char* stateName(RobotState state) {
  switch (state) {
    case STATE_INIT: return "INIT";
    case STATE_CALIBRATION: return "CALIBRATION";
    case STATE_LINE_TRACE: return "LINE_TRACE";
    case STATE_STRAIGHT_TRACE: return "STRAIGHT_TRACE";
    case STATE_CURVE_ENTRY: return "CURVE_ENTRY";
    case STATE_CURVE_TRACE: return "CURVE_TRACE";
    case STATE_LINE_RECOVERY: return "LINE_RECOVERY";
    case STATE_BACKTRACK: return "BACKTRACK";
    case STATE_OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
    case STATE_OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOIDANCE";
    case STATE_TILT_RECOVERY: return "TILT_RECOVERY";
    case STATE_SENSOR_TEST: return "SENSOR_TEST";
    case STATE_MOTOR_TEST: return "MOTOR_TEST";
    case STATE_SERVO_TEST: return "SERVO_TEST";
    case STATE_GOAL: return "GOAL";
    case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}
