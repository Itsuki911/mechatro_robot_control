/*
  dc_motor_mg1012_check.ino

  DCモータを低PWMから段階的に回し、回転開始PWM、停止、方向を確認する。
  ロボット本体コードとは独立した、Arduino IDE用の単体性能検証スケッチ。

  配線例:
    モータドライバ PWM -> D5
    モータドライバ DIR -> D7
    モータ電源 -> 外部電源
    モータドライバGND と Arduino GND を共通

  シリアルモニター:
    115200bps
*/

const int MOTOR_PWM_PIN = 5;
const int MOTOR_DIR_PIN = 7;
const int PWM_VALUES[] = {0, 45, 70, 95, 120, 0};
const int PWM_COUNT = sizeof(PWM_VALUES) / sizeof(PWM_VALUES[0]);
const unsigned long STEP_HOLD_MS = 1800;
const unsigned long STOP_HOLD_MS = 1200;

int stepIndex = 0;
int direction = HIGH;
unsigned long stepStartedMs = 0;

void applyMotor(int pwm, int dir) {
  digitalWrite(MOTOR_DIR_PIN, dir);
  analogWrite(MOTOR_PWM_PIN, constrain(pwm, 0, 255));

  Serial.print(millis());
  Serial.print(',');
  Serial.print(dir == HIGH ? F("forward") : F("reverse"));
  Serial.print(',');
  Serial.println(pwm);
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_DIR_PIN, direction);
  delay(1000);
  Serial.println(F("time_ms,direction,pwm"));
  applyMotor(PWM_VALUES[stepIndex], direction);
  stepStartedMs = millis();
}

void loop() {
  unsigned long now = millis();
  int currentPwm = PWM_VALUES[stepIndex];
  unsigned long holdMs = currentPwm == 0 ? STOP_HOLD_MS : STEP_HOLD_MS;

  if (now - stepStartedMs < holdMs) {
    return;
  }

  stepIndex++;
  if (stepIndex >= PWM_COUNT) {
    stepIndex = 0;
    direction = direction == HIGH ? LOW : HIGH;
  }

  applyMotor(PWM_VALUES[stepIndex], direction);
  stepStartedMs = now;
}
