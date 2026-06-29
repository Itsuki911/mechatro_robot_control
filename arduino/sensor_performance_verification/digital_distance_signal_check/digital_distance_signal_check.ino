/*
  digital_distance_signal_check.ino

  距離検知用のデジタル信号をD2で確認する。
  ロボット本体コードとは独立した、Arduino IDE用の単体性能検証スケッチ。

  重要:
    秋月電子のGP2Y0A21YK商品ページではインターフェイスがアナログとされています。
    そのため、GP2Y0A21YKのOUTをArduinoのデジタルピンへ直接つないでも距離cmは読めません。
    このスケッチは、比較器付きモジュールや外部回路で「近い/遠い」をHIGH/LOWに
    変換した信号を確認するためのコードです。

  配線:
    デジタル距離検知信号 OUT -> D2
    センサー/比較器 VCC/GND -> モジュール仕様に合わせて接続
    センサー/比較器 GND と Arduino GND を共通

  シリアルモニター:
    115200bps
*/

const int DISTANCE_DIGITAL_PIN = 2;
// モジュールによって、物体検知時がLOWの場合とHIGHの場合がある。
// 動作が逆なら、この値をLOWに変更する。
const int DETECTED_LEVEL = HIGH;
const unsigned long PRINT_INTERVAL_MS = 100;

unsigned long lastPrintMs = 0;
int previousState = -1;
unsigned long lastChangeMs = 0;

void setup() {
  Serial.begin(115200);
  // 外部モジュールが信号を明確にHIGH/LOWへ出す前提なのでINPUTにする。
  // 出力がオープンコレクタ等で不安定な場合はINPUT_PULLUPへ変更する。
  pinMode(DISTANCE_DIGITAL_PIN, INPUT);
  delay(1000);
  Serial.println(F("time_ms,digital_state,object_detected,last_change_ms"));
}

void loop() {
  unsigned long now = millis();
  int currentState = digitalRead(DISTANCE_DIGITAL_PIN);

  if (previousState < 0 || currentState != previousState) {
    previousState = currentState;
    lastChangeMs = now;
  }

  if (now - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMs = now;

  int objectDetected = currentState == DETECTED_LEVEL ? 1 : 0;

  // distance_cmは出さない。デジタル入力では距離ではなく検知/非検知だけを確認する。
  Serial.print(now);
  Serial.print(',');
  Serial.print(currentState);
  Serial.print(',');
  Serial.print(objectDetected);
  Serial.print(',');
  Serial.println(lastChangeMs);
}
