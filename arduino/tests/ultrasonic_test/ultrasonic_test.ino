/*
  ultrasonic_test.ino

  超音波センサー単体の距離確認用スケッチ。
  10cm、20cm、30cmなどでdistance_cmとokフラグを確認する。
*/

const int PIN_ULTRASONIC_TRIG = 8;
const int PIN_ULTRASONIC_ECHO = 9;
const unsigned long ULTRASONIC_TIMEOUT_US = 25000;

// TRIG/ECHOから距離をcmで読む。timeout時はok=falseにする。
float readDistanceCm(bool* ok) {
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

// シリアルと超音波センサーピンを初期化する。
void setup() {
  Serial.begin(115200);
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  Serial.println("time_ms,distance_cm,ok");
}

// 100msごとに距離とokフラグをCSVで出力する。
void loop() {
  bool ok = false;
  float cm = readDistanceCm(&ok);
  Serial.print(millis());
  Serial.print(',');
  Serial.print(cm, 1);
  Serial.print(',');
  Serial.println(ok ? 1 : 0);
  delay(100);
}
