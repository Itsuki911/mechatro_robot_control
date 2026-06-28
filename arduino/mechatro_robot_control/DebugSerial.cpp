#include "DebugSerial.h"

#include "Actuators.h"
#include "Config.h"
#include "Controller.h"

static unsigned long lastCsvMs = 0;
static unsigned long lastStateMs = 0;

void initDebugSerial() {
  Serial.begin(115200);
  Serial.println(F("time_ms,state,s1,s2,s3,s4,line_pos,line_error,straight_ms,curve_ms,distance_cm,roll_deg,servo_deg,drive_speed,estimated_distance_cm,error_flags,ultrasonic_ok,mpu_ok,color_ok,all_zero,event_code"));
}

void printCsvLog(const SensorData& sensor, const MotorCommand& command,
                 const ControllerTelemetry& telemetry, float estimatedDistanceCm) {
  if (sensor.timeMs - lastCsvMs < SERIAL_INTERVAL_MS) {
    return;
  }
  lastCsvMs = sensor.timeMs;
  unsigned int errors = telemetry.errorFlags | actuatorErrorFlags();

  Serial.print(sensor.timeMs);
  Serial.print(',');
  Serial.print(stateName(telemetry.state));
  Serial.print(',');
  for (int i = 0; i < 4; i++) {
    Serial.print(sensor.color[i]);
    Serial.print(',');
  }
  Serial.print(telemetry.linePosition);
  Serial.print(',');
  Serial.print(telemetry.lineError);
  Serial.print(',');
  Serial.print(telemetry.straightMs);
  Serial.print(',');
  Serial.print(telemetry.curveMs);
  Serial.print(',');
  Serial.print(sensor.distanceCm, 1);
  Serial.print(',');
  Serial.print(sensor.rollDeg, 1);
  Serial.print(',');
  Serial.print(currentServoDeg());
  Serial.print(',');
  Serial.print(currentDriveSpeed(), 3);
  Serial.print(',');
  Serial.print(estimatedDistanceCm, 2);
  Serial.print(',');
  Serial.print(errors);
  Serial.print(',');
  Serial.print(sensor.distanceOk ? 1 : 0);
  Serial.print(',');
  Serial.print(sensor.mpuOk ? 1 : 0);
  Serial.print(',');
  Serial.print(sensor.colorOk ? 1 : 0);
  Serial.print(',');
  Serial.print(sensor.allZero ? 1 : 0);
  Serial.print(',');
  Serial.println(telemetry.eventCode);
  (void)command;
}

void printStateSummary(const ControllerTelemetry& telemetry) {
  unsigned long nowMs = millis();
  if (nowMs - lastStateMs < STATE_LOG_INTERVAL_MS) {
    return;
  }
  lastStateMs = nowMs;
  Serial.print(F("# state="));
  Serial.print(stateName(telemetry.state));
  Serial.print(F(",servo="));
  Serial.print(currentServoDeg());
  Serial.print(F(",drive="));
  Serial.print(currentDriveSpeed(), 2);
  Serial.print(F(",errors="));
  Serial.println(telemetry.errorFlags);
}

void handleDebugCommands() {
  if (!Serial.available()) {
    return;
  }
  char c = Serial.read();
  if (c == 'e' || c == 'E') {
    stopActuators();
    Serial.println(F("# emergency_stop_command"));
  } else if (c == 'h' || c == 'H') {
    Serial.println(F("# commands: e=stop, h=help"));
  }
}
