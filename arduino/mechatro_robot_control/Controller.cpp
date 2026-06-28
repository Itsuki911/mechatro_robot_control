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

static void setState(RobotState next, unsigned long nowMs) {
  if (currentState != next) {
    currentState = next;
    stateEnteredMs = nowMs;
  }
}

static bool lineAt(const SensorData& sensor, int index) {
  return isLineDetectedValue(sensor.color[index]);
}

static bool floorAt(const SensorData& sensor, int index) {
  return sensor.allZero || isFloorDetectedValue(sensor.color[index]);
}

bool allLine(const SensorData& sensor) {
  return lineAt(sensor, 0) && lineAt(sensor, 1) && lineAt(sensor, 2) && lineAt(sensor, 3);
}

bool allFloor(const SensorData& sensor) {
  return floorAt(sensor, 0) && floorAt(sensor, 1) && floorAt(sensor, 2) && floorAt(sensor, 3);
}

static int calcLineError(const SensorData& sensor) {
  const int weights[4] = {-3, -1, 1, 3};
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

static int calcLinePosition(const SensorData& sensor) {
  const int positions[4] = {-30, -10, 10, 30};
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

static MotorCommand stopCommand() {
  MotorCommand cmd;
  cmd.driveSpeed = 0.0;
  cmd.servoDeg = SERVO_CENTER;
  cmd.stop = true;
  cmd.reverse = false;
  return cmd;
}

static MotorCommand driveCommand(float speed, int servoDeg) {
  MotorCommand cmd;
  cmd.driveSpeed = speed;
  cmd.servoDeg = servoDeg;
  cmd.stop = speed == 0.0;
  cmd.reverse = speed < 0.0;
  return cmd;
}

static void updateLineTimers(const SensorData& sensor, unsigned long nowMs) {
  bool centerLine = lineAt(sensor, 1) || lineAt(sensor, 2);
  bool leftCurveLine = lineAt(sensor, 0) || (lineAt(sensor, 0) && lineAt(sensor, 1));
  bool rightCurveLine = lineAt(sensor, 3) || (lineAt(sensor, 2) && lineAt(sensor, 3));

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

  if (!sensor.mpuOk || abs(sensor.rollDeg) >= TILT_RECOVER_DEG) {
    setState(STATE_TILT_RECOVERY, nowMs);
  } else if (sensor.distanceOk && sensor.distanceCm <= OBSTACLE_DISTANCE_CM) {
    setState(STATE_OBSTACLE_DETECTED, nowMs);
  }

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

ControllerTelemetry getControllerTelemetry() {
  telemetry.state = currentState;
  return telemetry;
}

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
