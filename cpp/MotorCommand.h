/*
  MotorCommand.h

  PC上の制御デモが返す出力指令。
  実機と同じく、後輪DCモーター速度と前輪サーボ角を中心にしている。
*/

#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

// driveSpeedは-1.0..1.0相当、servoDegは90度を直進中心とする。
struct MotorCommand {
  float driveSpeed = 0.0f;
  int servoDeg = 90;
  bool stop = true;
  bool reverse = false;
};

#endif
