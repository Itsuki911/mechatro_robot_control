/*
  Actuators.cpp

  サーボと後輪DCモーターへ実際に出力するファイル。
  Controller.cppは「目標値」を決めるだけにして、このファイルで急な角度変化や
  急加速を抑える。実機破損を避けるため、出力範囲外の指令は丸めてエラーに残す。
*/

#include "Actuators.h"

#include <Servo.h>

#include "Config.h"

static Servo steeringServo;
static float driveNow = 0.0;
static int servoNow = SERVO_CENTER;
static unsigned int errors = ERROR_NONE;

// 現在値を目標値へ少しだけ近づける。速度の急変化を防ぐために使う。
static float rampFloat(float current, float target, float step) {
  if (target > current + step) {
    return current + step;
  }
  if (target < current - step) {
    return current - step;
  }
  return target;
}

// サーボ角度用の整数版ランプ処理。1周期でMAX_SERVO_STEP度までしか動かさない。
static int rampInt(int current, int target, int step) {
  if (target > current + step) {
    return current + step;
  }
  if (target < current - step) {
    return current - step;
  }
  return target;
}

// -1.0..1.0の駆動指令を、DIRピンとPWM値へ変換する。
static void writeDriveMotor(float value) {
  if (value > 1.0 || value < -1.0) {
    errors |= ERROR_MOTOR_RANGE;
  }
  value = constrain(value, -1.0, 1.0);
  bool forward = value >= 0.0;
  int pwm = (int)(abs(value) * 255.0);
  digitalWrite(PIN_DRIVE_DIR, forward ? HIGH : LOW);
  analogWrite(PIN_DRIVE_PWM, pwm);
}

// サーボ、DCモーターの出力ピンを初期化し、起動直後は必ず停止状態にする。
void initActuators() {
  pinMode(PIN_DRIVE_PWM, OUTPUT);
  pinMode(PIN_DRIVE_DIR, OUTPUT);
  steeringServo.attach(PIN_SERVO);
  stopActuators();
}

// Controllerから受け取った目標指令を、安全な変化量へ制限して実機へ出す。
void applyMotorCommand(const MotorCommand& command) {
  int targetServo = command.servoDeg;
  if (targetServo < SERVO_LEFT_MAX || targetServo > SERVO_RIGHT_MAX) {
    errors |= ERROR_SERVO_RANGE;
  }
  targetServo = constrain(targetServo, SERVO_LEFT_MAX, SERVO_RIGHT_MAX);
  servoNow = rampInt(servoNow, targetServo, MAX_SERVO_STEP);
  steeringServo.write(servoNow);

  if (command.stop) {
    driveNow = rampFloat(driveNow, 0.0, MAX_SPEED_STEP);
  } else {
    driveNow = rampFloat(driveNow, command.driveSpeed, MAX_SPEED_STEP);
  }
  writeDriveMotor(driveNow);
}

// Emergency Stopや初期化で使う即時停止処理。記録上の現在値も0/中央へ戻す。
void stopActuators() {
  driveNow = 0.0;
  servoNow = SERVO_CENTER;
  steeringServo.write(servoNow);
  analogWrite(PIN_DRIVE_PWM, 0);
  digitalWrite(PIN_DRIVE_DIR, LOW);
}

// CSVログ用に、ランプ処理後の実際に近い駆動値を返す。
float currentDriveSpeed() {
  return driveNow;
}

// CSVログ用に、ランプ処理後の実際に近いサーボ角を返す。
int currentServoDeg() {
  return servoNow;
}

// Actuators内で検出した範囲外指令を返し、次周期へ持ち越さないようクリアする。
unsigned int actuatorErrorFlags() {
  unsigned int value = errors;
  errors = ERROR_NONE;
  return value;
}
