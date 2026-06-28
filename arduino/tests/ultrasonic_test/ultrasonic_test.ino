const int PIN_ULTRASONIC_TRIG = 8;
const int PIN_ULTRASONIC_ECHO = 9;
const unsigned long ULTRASONIC_TIMEOUT_US = 25000;

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

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  Serial.println("time_ms,distance_cm,ok");
}

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
