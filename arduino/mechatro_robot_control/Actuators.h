#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "Types.h"

void initActuators();
void applyMotorCommand(const MotorCommand& command);
void stopActuators();
float currentDriveSpeed();
int currentServoDeg();
unsigned int actuatorErrorFlags();

#endif
