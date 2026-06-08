#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

struct MotorCommand {
  // メカナム4輪の指令。範囲は -1.0 .. 1.0。
  float frontLeft = 0.0f;
  float frontRight = 0.0f;
  float rearLeft = 0.0f;
  float rearRight = 0.0f;

  // サーボ角度。90度を直進中心とする仮定。
  int servoDeg = 90;

  bool stop = false;
};

#endif
