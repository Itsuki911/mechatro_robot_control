/*
  mechatro_robot_control.ino

  Arduinoスケッチの入口。
  setup()で各モジュールを初期化し、loop()では「読む -> 判断する -> 出す -> 記録する」
  の順番だけを管理する。細かい処理は各.cppへ分け、Arduino IDEのタブでも追いやすくする。
*/

#include "Actuators.h"
#include "Config.h"
#include "Controller.h"
#include "DebugSerial.h"
#include "Sensors.h"

static unsigned long lastControlMs = 0;
static unsigned long lastDistanceUpdateMs = 0;
static float estimatedDistanceCm = 0.0;

// エンコーダがないため、駆動指令と経過時間から大まかな移動距離だけを積算する。
static void updateEstimatedDistance(float driveSpeed, unsigned long nowMs) {
  if (lastDistanceUpdateMs == 0) {
    lastDistanceUpdateMs = nowMs;
    return;
  }
  unsigned long dtMs = nowMs - lastDistanceUpdateMs;
  lastDistanceUpdateMs = nowMs;
  float seconds = dtMs / 1000.0;
  estimatedDistanceCm += abs(driveSpeed) * ESTIMATED_SPEED_CM_PER_SEC_AT_FULL_PWM * seconds;
}

// 電源投入後に1回だけ実行される初期化処理。
void setup() {
  initDebugSerial();
  initSensors();
  initActuators();
  initController();
  Serial.println(F("# mechatro_robot_control servo_rear_dc start"));
}

// 約20msごとに制御を進める。delay()ではなく時刻差で周期を作る。
void loop() {
  unsigned long nowMs = millis();
  handleDebugCommands();
  if (nowMs - lastControlMs < CONTROL_INTERVAL_MS) {
    return;
  }
  lastControlMs = nowMs;

  SensorData sensor = readSensors(nowMs);
  MotorCommand command = updateController(sensor);
  applyMotorCommand(command);
  updateEstimatedDistance(currentDriveSpeed(), nowMs);

  ControllerTelemetry telemetry = getControllerTelemetry();
  printCsvLog(sensor, command, telemetry, estimatedDistanceCm);
  printStateSummary(telemetry);
}
