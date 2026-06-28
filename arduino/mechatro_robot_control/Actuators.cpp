#include "Actuators.h"

#include <Servo.h>

#include "Config.h"

static Servo steeringServo;
static float driveNow = 0.0;
static int servoNow = SERVO_CENTER;
static unsigned int errors = ERROR_NONE;

static float rampFloat(float current, float target, float step) {
  if (target > current + step) {
    return current + step;
  }
  if (target < current - step) {
    return current - step;
  }
  return target;
}

static int rampInt(int current, int target, int step) {
  if (target > current + step) {
    return current + step;
  }
  if (target < current - step) {
    return current - step;
  }
  return target;
}

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

void initActuators() {
  pinMode(PIN_DRIVE_PWM, OUTPUT);
  pinMode(PIN_DRIVE_DIR, OUTPUT);
  steeringServo.attach(PIN_SERVO);
  stopActuators();
}

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

void stopActuators() {
  driveNow = 0.0;
  servoNow = SERVO_CENTER;
  steeringServo.write(servoNow);
  analogWrite(PIN_DRIVE_PWM, 0);
  digitalWrite(PIN_DRIVE_DIR, LOW);
}

float currentDriveSpeed() {
  return driveNow;
}

int currentServoDeg() {
  return servoNow;
}

unsigned int actuatorErrorFlags() {
  unsigned int value = errors;
  errors = ERROR_NONE;
  return value;
}
