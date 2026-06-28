#include "Sensors.h"

#include <Wire.h>
#include <math.h>

#include "Config.h"

static int previousColor[4] = {0, 0, 0, 0};
static unsigned long lastColorChangeMs = 0;

static void selectMux(int channel) {
  digitalWrite(PIN_MUX_S0, channel & 0x01);
  digitalWrite(PIN_MUX_S1, (channel >> 1) & 0x01);
  digitalWrite(PIN_MUX_S2, (channel >> 2) & 0x01);
}

static int readMuxAverage(int channel) {
  selectMux(channel);
  delayMicroseconds(180);
  long sum = 0;
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    sum += analogRead(PIN_MUX_SIG);
    delayMicroseconds(120);
  }
  return (int)(sum / SENSOR_SAMPLES);
}

static void initMpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

static float readRollDeg(bool* ok) {
  *ok = true;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    *ok = false;
    return 0.0;
  }
  Wire.requestFrom(MPU_ADDR, (byte)6, (byte)true);
  if (Wire.available() < 6) {
    *ok = false;
    return 0.0;
  }
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  (void)ax;
  float ayG = ay / 16384.0;
  float azG = az / 16384.0;
  return atan2(ayG, azG) * 57.2958;
}

void initSensors() {
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  Wire.begin();
  initMpu();
  lastColorChangeMs = millis();
}

SensorData readSensors(unsigned long nowMs) {
  SensorData data;
  data.timeMs = nowMs;
  data.color[0] = readMuxAverage(CH_COLOR_S1);
  data.color[1] = readMuxAverage(CH_COLOR_S2);
  data.color[2] = readMuxAverage(CH_COLOR_S3);
  data.color[3] = readMuxAverage(CH_COLOR_S4);
  data.distanceCm = readUltrasonicDistanceCm(&data.distanceOk);
  data.rollDeg = readRollDeg(&data.mpuOk);
  data.errorFlags = ERROR_NONE;

  data.allZero = true;
  bool changed = false;
  for (int i = 0; i < 4; i++) {
    if (data.color[i] != 0) {
      data.allZero = false;
    }
    if (abs(data.color[i] - previousColor[i]) > SENSOR_STUCK_DELTA) {
      changed = true;
    }
    previousColor[i] = data.color[i];
  }
  if (changed) {
    lastColorChangeMs = nowMs;
  }

  data.colorOk = !data.allZero;
  if (data.allZero) {
    data.errorFlags |= ERROR_COLOR_ALL_ZERO;
  }
  if (!data.distanceOk) {
    data.errorFlags |= ERROR_DISTANCE_TIMEOUT;
  }
  if (!data.mpuOk) {
    data.errorFlags |= ERROR_MPU_READ;
  }
  if (nowMs - lastColorChangeMs >= SENSOR_STUCK_MS) {
    data.errorFlags |= ERROR_SENSOR_STUCK;
  }
  return data;
}

LineColor classifyLineValue(int value) {
  if (LINE_IS_WHITE) {
    if (value >= WHITE_LINE_THRESHOLD) {
      return LINE_DETECTED;
    }
    if (value <= BLACK_FLOOR_THRESHOLD) {
      return LINE_FLOOR;
    }
  } else {
    if (value <= BLACK_FLOOR_THRESHOLD) {
      return LINE_DETECTED;
    }
    if (value >= WHITE_LINE_THRESHOLD) {
      return LINE_FLOOR;
    }
  }
  return LINE_UNKNOWN;
}

bool isLineDetectedValue(int value) {
  return classifyLineValue(value) == LINE_DETECTED;
}

bool isFloorDetectedValue(int value) {
  return classifyLineValue(value) == LINE_FLOOR;
}

float readUltrasonicDistanceCm(bool* ok) {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    *ok = false;
    return 999.0;
  }
  *ok = true;
  return duration / 58.0;
}
