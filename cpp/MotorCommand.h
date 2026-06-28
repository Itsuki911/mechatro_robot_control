#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

struct MotorCommand {
  float driveSpeed = 0.0f;
  int servoDeg = 90;
  bool stop = true;
  bool reverse = false;
};

#endif
