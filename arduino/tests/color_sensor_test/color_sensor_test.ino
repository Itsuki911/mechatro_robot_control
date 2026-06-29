/*
  color_sensor_test.ino

  A0-A3へ直結したカラーセンサーS1-S4のG出力をCSVで出すテスト。
  白線上と黒床上の値を記録し、Config.hの閾値を決めるために使う。
*/

const int COLOR_PINS[4] = {A0, A1, A2, A3};

// シリアルとカラーセンサー入力ピンを初期化する。
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(COLOR_PINS[i], INPUT);
  }
  Serial.println("time_ms,s1,s2,s3,s4");
}

// 100msごとにS1-S4を1行のCSVとして出力する。
void loop() {
  Serial.print(millis());
  for (int i = 0; i < 4; i++) {
    Serial.print(',');
    Serial.print(analogRead(COLOR_PINS[i]));
  }
  Serial.println();
  delay(100);
}
