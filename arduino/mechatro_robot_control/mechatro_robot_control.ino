#include <Servo.h>
#include <Wire.h>

/*
  Mechatro robot control

  重要な仮定:
  - 指定資料名 robot_arcitecture.pptx.pdf は見つからず、Downloads内の
    robot_architecture.pptx.pdf を確認した。
  - フローチャートはS1..S4の4個ラインセンサーを前提としている。
  - Arduino UNOでは4個カラーセンサー + 距離センサー + MPU(I2C)を直結する
    アナログ入力が不足するため、74HC4051等のアナログマルチプレクサを使う前提。
  - S9032-02はフォトダイオード素子なので、本来は抵抗またはOPアンプで
    電圧化した出力をanalogReadする。ここでは「黒ほど値が小さい」と仮定。
  - メカナム4輪はDRV8835を2枚使い、PH/EN相当のPWM+DIRで各DCモーターを制御する。
*/

// ===== ピン設定: 実機配線に合わせて最初に確認する =====
const int PIN_MUX_SIG = A0;     // 74HC4051の共通出力
const int PIN_MUX_S0 = 2;
const int PIN_MUX_S1 = 4;
const int PIN_MUX_S2 = A1;      // アナログ端子をデジタル出力として使用

const int PIN_SERVO = 10;

// ServoライブラリはUNOのTimer1を使うため、D9/D10のPWMは避ける。
const int PIN_FL_PWM = 3;       // Front Left
const int PIN_FL_DIR = 7;
const int PIN_FR_PWM = 5;       // Front Right
const int PIN_FR_DIR = 8;
const int PIN_RL_PWM = 6;       // Rear Left
const int PIN_RL_DIR = 12;
const int PIN_RR_PWM = 11;      // Rear Right
const int PIN_RR_DIR = 13;

const byte MPU_ADDR = 0x68;

// マルチプレクサのチャンネル割当
const int CH_COLOR_S1 = 0;
const int CH_COLOR_S2 = 1;
const int CH_COLOR_S3 = 2;
const int CH_COLOR_S4 = 3;
const int CH_DISTANCE = 4;

// ===== 調整パラメータ: 実機テストで必ず変更する =====
const int BLACK_THRESHOLD = 430;       // これ未満を黒
const int WHITE_THRESHOLD = 720;       // これ以上を白
const int SENSOR_SAMPLES = 5;

const float BASE_SPEED = 0.28;         // 初期は低速
const float CURVE_SPEED = 0.22;
const float RECOVERY_SPEED = 0.18;
const float MAX_SPEED_STEP = 0.035;    // 急加速防止

const float OBSTACLE_DISTANCE_CM = 18.0;
const float TILT_RECOVER_DEG = 15.0;
const float TILT_STOP_DEG = 30.0;

const unsigned long CALIBRATION_MS = 2500;
const unsigned long GOAL_CONFIRM_MS = 700;
const unsigned long OBSTACLE_AVOID_MS = 900;
const unsigned long SERIAL_INTERVAL_MS = 200;

const int SERVO_CENTER = 90;
const int SERVO_LEFT = 60;
const int SERVO_RIGHT = 120;

// モーター配線が逆なら該当値を -1 にする
const int FL_DIRECTION_SIGN = 1;
const int FR_DIRECTION_SIGN = 1;
const int RL_DIRECTION_SIGN = 1;
const int RR_DIRECTION_SIGN = 1;

enum RobotState {
  STATE_INIT,
  STATE_CALIBRATION,
  STATE_LINE_TRACE,
  STATE_CURVE_ENTRY,
  STATE_CURVE_TRACE,
  STATE_OBSTACLE_DETECTED,
  STATE_OBSTACLE_AVOIDANCE,
  STATE_LINE_RECOVERY,
  STATE_TILT_RECOVERY,
  STATE_GOAL,
  STATE_EMERGENCY_STOP
};

enum LineColor {
  LINE_BLACK,
  LINE_WHITE,
  LINE_OTHER
};

struct SensorData {
  int color[4];
  float distanceCm;
  float rollDeg;
};

struct MotorCommand {
  float fl;
  float fr;
  float rl;
  float rr;
  int servoDeg;
  bool stop;
};

Servo steeringServo;
RobotState state = STATE_INIT;
unsigned long stateEnteredMs = 0;
unsigned long goalCandidateMs = 0;
unsigned long lastSerialMs = 0;
int lastLineError = 0;
float currentFl = 0.0;
float currentFr = 0.0;
float currentRl = 0.0;
float currentRr = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);

  pinMode(PIN_FL_PWM, OUTPUT);
  pinMode(PIN_FL_DIR, OUTPUT);
  pinMode(PIN_FR_PWM, OUTPUT);
  pinMode(PIN_FR_DIR, OUTPUT);
  pinMode(PIN_RL_PWM, OUTPUT);
  pinMode(PIN_RL_DIR, OUTPUT);
  pinMode(PIN_RR_PWM, OUTPUT);
  pinMode(PIN_RR_DIR, OUTPUT);

  steeringServo.attach(PIN_SERVO);
  steeringServo.write(SERVO_CENTER);
  stopMotors();
  initMpu();

  setState(STATE_CALIBRATION);
  Serial.println(F("mechatro_robot_control start"));
}

void loop() {
  SensorData sensor = readSensors();

  if (abs(sensor.rollDeg) >= TILT_STOP_DEG) {
    setState(STATE_EMERGENCY_STOP);
  }

  MotorCommand command = updateController(sensor);
  applyMotorCommand(command);
  printDebug(sensor, command);

  delay(20);  // 約50Hz制御。センサーが不安定なら30-40msへ増やす。
}

MotorCommand updateController(const SensorData& sensor) {
  if (state == STATE_EMERGENCY_STOP || state == STATE_GOAL) {
    return makeStopCommand();
  }

  if (state == STATE_CALIBRATION) {
    if (millis() - stateEnteredMs >= CALIBRATION_MS) {
      setState(STATE_LINE_TRACE);
    }
    return makeStopCommand();
  }

  if (abs(sensor.rollDeg) >= TILT_RECOVER_DEG) {
    setState(STATE_TILT_RECOVERY);
  } else if (sensor.distanceCm > 0.0 && sensor.distanceCm <= OBSTACLE_DISTANCE_CM) {
    setState(STATE_OBSTACLE_DETECTED);
  }

  if (isGoalPattern(sensor)) {
    if (goalCandidateMs == 0) {
      goalCandidateMs = millis();
    }
    if (millis() - goalCandidateMs >= GOAL_CONFIRM_MS) {
      setState(STATE_GOAL);
      return makeStopCommand();
    }
  } else {
    goalCandidateMs = 0;
  }

  if (state == STATE_TILT_RECOVERY) {
    if (abs(sensor.rollDeg) < TILT_RECOVER_DEG * 0.6) {
      setState(STATE_LINE_TRACE);
    }
    float omega = sensor.rollDeg > 0.0 ? -0.18 : 0.18;
    return makeMecanumCommand(RECOVERY_SPEED * 0.5, 0.0, omega, SERVO_CENTER);
  }

  if (state == STATE_OBSTACLE_DETECTED) {
    setState(STATE_OBSTACLE_AVOIDANCE);
    return makeStopCommand();
  }

  if (state == STATE_OBSTACLE_AVOIDANCE) {
    if (millis() - stateEnteredMs >= OBSTACLE_AVOID_MS) {
      setState(STATE_LINE_RECOVERY);
    }
    // 右へ横移動しつつ低速前進。障害物が固定の場合は距離閾値と時間を短めに調整する。
    return makeMecanumCommand(RECOVERY_SPEED, -0.18, 0.10, SERVO_RIGHT);
  }

  if (state == STATE_LINE_RECOVERY) {
    if (!allWhite(sensor)) {
      setState(STATE_LINE_TRACE);
    }
    float omega = lastLineError >= 0 ? -0.22 : 0.22;
    int servo = lastLineError >= 0 ? SERVO_LEFT : SERVO_RIGHT;
    return makeMecanumCommand(RECOVERY_SPEED, 0.0, omega, servo);
  }

  if (allWhite(sensor)) {
    setState(STATE_LINE_RECOVERY);
    return makeMecanumCommand(0.0, 0.0, lastLineError >= 0 ? -0.18 : 0.18, SERVO_CENTER);
  }

  bool s1 = isBlack(sensor, 0);
  bool s2 = isBlack(sensor, 1);
  bool s3 = isBlack(sensor, 2);
  bool s4 = isBlack(sensor, 3);

  if ((s1 || s4) && !(s2 && s3)) {
    setState(STATE_CURVE_ENTRY);
  }
  if ((s1 && s4) && !s2 && !s3) {
    // フローチャートの「S1&S4黒、S2&S3白」はカーブ完了/通常復帰と解釈。
    setState(STATE_LINE_TRACE);
  } else if (state == STATE_CURVE_ENTRY) {
    setState(STATE_CURVE_TRACE);
  }

  int error = calcLineError(sensor);
  lastLineError = error;
  float gain = (state == STATE_CURVE_TRACE) ? 0.085 : 0.055;
  float speed = (state == STATE_CURVE_TRACE) ? CURVE_SPEED : BASE_SPEED;
  float omega = constrainFloat(-gain * error, -0.45, 0.45);
  int servo = constrain(SERVO_CENTER - error * 7, SERVO_LEFT, SERVO_RIGHT);

  return makeMecanumCommand(speed, 0.0, omega, servo);
}

SensorData readSensors() {
  SensorData data;
  data.color[0] = readMuxAverage(CH_COLOR_S1);
  data.color[1] = readMuxAverage(CH_COLOR_S2);
  data.color[2] = readMuxAverage(CH_COLOR_S3);
  data.color[3] = readMuxAverage(CH_COLOR_S4);
  data.distanceCm = readDistanceCm();
  data.rollDeg = readRollDeg();
  return data;
}

int readMuxAverage(int channel) {
  selectMux(channel);
  delayMicroseconds(250);
  long sum = 0;
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    sum += analogRead(PIN_MUX_SIG);
    delayMicroseconds(200);
  }
  return (int)(sum / SENSOR_SAMPLES);
}

void selectMux(int channel) {
  digitalWrite(PIN_MUX_S0, channel & 0x01);
  digitalWrite(PIN_MUX_S1, (channel >> 1) & 0x01);
  digitalWrite(PIN_MUX_S2, (channel >> 2) & 0x01);
}

float readDistanceCm() {
  int raw = readMuxAverage(CH_DISTANCE);
  float voltage = raw * (5.0 / 1023.0);
  // GP2Y0A21YKの概算式。10-80cm付近以外では誤差が大きいので実機で表を作る。
  if (voltage <= 0.42) {
    return 80.0;
  }
  float cm = 27.86 * pow(voltage, -1.15);
  return constrainFloat(cm, 6.0, 80.0);
}

void initMpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // wake up
  Wire.endTransmission(true);
}

float readRollDeg() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    return 0.0;      // MPU未接続時は停止させたい場合、ここで大角度を返す。
  }
  Wire.requestFrom(MPU_ADDR, (byte)6, (byte)true);
  if (Wire.available() < 6) {
    return 0.0;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  float ayG = ay / 16384.0;
  float azG = az / 16384.0;
  float roll = atan2(ayG, azG) * 57.2958;
  (void)ax;
  return roll;
}

LineColor classifyLine(int value) {
  if (value <= BLACK_THRESHOLD) {
    return LINE_BLACK;
  }
  if (value >= WHITE_THRESHOLD) {
    return LINE_WHITE;
  }
  return LINE_OTHER;
}

bool isBlack(const SensorData& sensor, int index) {
  return classifyLine(sensor.color[index]) == LINE_BLACK;
}

bool allWhite(const SensorData& sensor) {
  for (int i = 0; i < 4; i++) {
    if (classifyLine(sensor.color[i]) != LINE_WHITE) {
      return false;
    }
  }
  return true;
}

bool isGoalPattern(const SensorData& sensor) {
  // コース図のゴールはスタート付近の横線。4センサー同時黒が一定時間続いたら停止。
  return isBlack(sensor, 0) && isBlack(sensor, 1) && isBlack(sensor, 2) && isBlack(sensor, 3);
}

int calcLineError(const SensorData& sensor) {
  const int weights[4] = {-3, -1, 1, 3};  // S1,S2,S3,S4を左から右へ並べた仮定。
  int sum = 0;
  int count = 0;
  for (int i = 0; i < 4; i++) {
    if (isBlack(sensor, i)) {
      sum += weights[i];
      count++;
    }
  }
  if (count == 0) {
    return lastLineError;
  }
  return sum / count;
}

MotorCommand makeStopCommand() {
  MotorCommand cmd;
  cmd.fl = 0.0;
  cmd.fr = 0.0;
  cmd.rl = 0.0;
  cmd.rr = 0.0;
  cmd.servoDeg = SERVO_CENTER;
  cmd.stop = true;
  return cmd;
}

MotorCommand makeMecanumCommand(float vx, float vy, float omega, int servoDeg) {
  MotorCommand cmd;
  cmd.fl = vx + vy + omega;
  cmd.fr = vx - vy - omega;
  cmd.rl = vx - vy + omega;
  cmd.rr = vx + vy - omega;

  float maxAbs = max(1.0, max(max(abs(cmd.fl), abs(cmd.fr)), max(abs(cmd.rl), abs(cmd.rr))));
  cmd.fl /= maxAbs;
  cmd.fr /= maxAbs;
  cmd.rl /= maxAbs;
  cmd.rr /= maxAbs;
  cmd.servoDeg = servoDeg;
  cmd.stop = false;
  return cmd;
}

void applyMotorCommand(const MotorCommand& cmd) {
  steeringServo.write(constrain(cmd.servoDeg, SERVO_LEFT, SERVO_RIGHT));

  if (cmd.stop) {
    stopMotors();
    return;
  }

  currentFl = ramp(currentFl, cmd.fl);
  currentFr = ramp(currentFr, cmd.fr);
  currentRl = ramp(currentRl, cmd.rl);
  currentRr = ramp(currentRr, cmd.rr);

  writeMotor(PIN_FL_PWM, PIN_FL_DIR, currentFl * FL_DIRECTION_SIGN);
  writeMotor(PIN_FR_PWM, PIN_FR_DIR, currentFr * FR_DIRECTION_SIGN);
  writeMotor(PIN_RL_PWM, PIN_RL_DIR, currentRl * RL_DIRECTION_SIGN);
  writeMotor(PIN_RR_PWM, PIN_RR_DIR, currentRr * RR_DIRECTION_SIGN);
}

float ramp(float current, float target) {
  if (target > current + MAX_SPEED_STEP) {
    return current + MAX_SPEED_STEP;
  }
  if (target < current - MAX_SPEED_STEP) {
    return current - MAX_SPEED_STEP;
  }
  return target;
}

void writeMotor(int pwmPin, int dirPin, float value) {
  value = constrainFloat(value, -1.0, 1.0);
  bool forward = value >= 0.0;
  int pwm = (int)(abs(value) * 255.0);
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  currentFl = 0.0;
  currentFr = 0.0;
  currentRl = 0.0;
  currentRr = 0.0;
  analogWrite(PIN_FL_PWM, 0);
  analogWrite(PIN_FR_PWM, 0);
  analogWrite(PIN_RL_PWM, 0);
  analogWrite(PIN_RR_PWM, 0);
}

void setState(RobotState next) {
  if (state != next) {
    state = next;
    stateEnteredMs = millis();
  }
}

float constrainFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

const char* stateName(RobotState value) {
  switch (value) {
    case STATE_INIT: return "INIT";
    case STATE_CALIBRATION: return "CALIBRATION";
    case STATE_LINE_TRACE: return "LINE_TRACE";
    case STATE_CURVE_ENTRY: return "CURVE_ENTRY";
    case STATE_CURVE_TRACE: return "CURVE_TRACE";
    case STATE_OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
    case STATE_OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOIDANCE";
    case STATE_LINE_RECOVERY: return "LINE_RECOVERY";
    case STATE_TILT_RECOVERY: return "TILT_RECOVERY";
    case STATE_GOAL: return "GOAL";
    case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

void printDebug(const SensorData& sensor, const MotorCommand& cmd) {
  if (millis() - lastSerialMs < SERIAL_INTERVAL_MS) {
    return;
  }
  lastSerialMs = millis();

  Serial.print(F("state="));
  Serial.print(stateName(state));
  Serial.print(F(" S="));
  for (int i = 0; i < 4; i++) {
    Serial.print(sensor.color[i]);
    Serial.print(i == 3 ? ' ' : ',');
  }
  Serial.print(F("dist="));
  Serial.print(sensor.distanceCm, 1);
  Serial.print(F(" roll="));
  Serial.print(sensor.rollDeg, 1);
  Serial.print(F(" motor="));
  Serial.print(cmd.fl, 2);
  Serial.print(',');
  Serial.print(cmd.fr, 2);
  Serial.print(',');
  Serial.print(cmd.rl, 2);
  Serial.print(',');
  Serial.print(cmd.rr, 2);
  Serial.print(F(" servo="));
  Serial.println(cmd.servoDeg);
}
