/*
  color_sensor_g_output_check.ino

  4個のカラーセンサーのG出力だけをA0-A3へ接続し、白線/黒床の判定材料を確認する。
  ロボット本体コードとは独立した、Arduino IDE用の単体性能検証スケッチ。

  配線:
    S1 G -> A0
    S2 G -> A1
    S3 G -> A2
    S4 G -> A3
    各センサー VCC/GND -> センサー仕様に合わせて接続

  シリアルモニター:
    115200bps
*/

const int SENSOR_COUNT = 4;
// 本体のひし形配置と同じ順番。S1=前, S2=左, S3=右, S4=後ろ。
const int SENSOR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3};
// 仮の白線判定しきい値。実測した白線値と黒床値の中間に調整する。
const int WHITE_THRESHOLD = 700;
// 複数回読んで平均することで、照明やADCの小さな揺れを見やすくする。
const int SAMPLES = 8;
const unsigned long PRINT_INTERVAL_MS = 100;

unsigned long lastPrintMs = 0;

int readAverage(int pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(pin);
    // 連続読み取り時にADC値が落ち着くように短く待つ。
    delayMicroseconds(300);
  }
  return (int)(sum / SAMPLES);
}

void printHeader() {
  Serial.println(F("time_ms,s1_raw,s2_raw,s3_raw,s4_raw,s1_line,s2_line,s3_line,s4_line"));
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
  delay(1000);
  printHeader();
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMs = now;

  int raw[SENSOR_COUNT];
  for (int i = 0; i < SENSOR_COUNT; i++) {
    raw[i] = readAverage(SENSOR_PINS[i]);
  }

  // CSV形式で出す。Arduino IDEのシリアルモニターでも、コピーして表計算にも貼れる。
  Serial.print(now);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(',');
    Serial.print(raw[i]);
  }
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(',');
    Serial.print(raw[i] >= WHITE_THRESHOLD ? 1 : 0);
  }
  Serial.println();
}
