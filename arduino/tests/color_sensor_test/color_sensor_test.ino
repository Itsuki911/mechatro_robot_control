const int PIN_MUX_SIG = A0;
const int PIN_MUX_S0 = 2;
const int PIN_MUX_S1 = 4;
const int PIN_MUX_S2 = A1;

void selectMux(int ch) {
  digitalWrite(PIN_MUX_S0, ch & 1);
  digitalWrite(PIN_MUX_S1, (ch >> 1) & 1);
  digitalWrite(PIN_MUX_S2, (ch >> 2) & 1);
}

int readMux(int ch) {
  selectMux(ch);
  delayMicroseconds(200);
  return analogRead(PIN_MUX_SIG);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  Serial.println("time_ms,s1,s2,s3,s4");
}

void loop() {
  Serial.print(millis());
  for (int ch = 0; ch < 4; ch++) {
    Serial.print(',');
    Serial.print(readMux(ch));
  }
  Serial.println();
  delay(100);
}
