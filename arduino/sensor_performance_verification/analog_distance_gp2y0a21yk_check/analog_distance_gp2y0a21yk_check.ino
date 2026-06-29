/*
  analog_distance_gp2y0a21yk_check.ino

  GP2Y0A21YK系アナログ距離モジュールの出力を確認する。
  ロボット本体コードとは独立した、Arduino IDE用の単体性能検証スケッチ。

  配線:
    距離モジュール OUT -> A0
    距離モジュール VCC/GND -> モジュール仕様に合わせて接続

  シリアルモニター:
    115200bps

  注意:
    距離換算は概算。最終的な判定閾値は実測値で決める。
*/

const int DISTANCE_PIN = A0;
const int SAMPLES = 16;
const unsigned long PRINT_INTERVAL_MS = 100;
const float ADC_REF_VOLTAGE = 5.0;
const float ADC_MAX = 1023.0;

unsigned long lastPrintMs = 0;

int readAverageRaw() {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(DISTANCE_PIN);
    delayMicroseconds(500);
  }
  return (int)(sum / SAMPLES);
}

float rawToVoltage(int raw) {
  return raw * ADC_REF_VOLTAGE / ADC_MAX;
}

float voltageToDistanceCm(float voltage) {
  if (voltage < 0.4) {
    return -1.0;
  }
  // GP2Y0A21YKの一般的な近似式。実測で補正して使う。
  float distance = 27.86 * pow(voltage, -1.15);
  if (distance < 0.0 || distance > 150.0) {
    return -1.0;
  }
  return distance;
}

void setup() {
  Serial.begin(115200);
  pinMode(DISTANCE_PIN, INPUT);
  delay(1000);
  Serial.println(F("time_ms,raw,voltage,distance_cm,valid"));
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMs = now;

  int raw = readAverageRaw();
  float voltage = rawToVoltage(raw);
  float distanceCm = voltageToDistanceCm(voltage);
  int valid = distanceCm > 0.0 ? 1 : 0;

  Serial.print(now);
  Serial.print(',');
  Serial.print(raw);
  Serial.print(',');
  Serial.print(voltage, 3);
  Serial.print(',');
  Serial.print(distanceCm, 1);
  Serial.print(',');
  Serial.println(valid);
}
