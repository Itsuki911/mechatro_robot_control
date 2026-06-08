# ピン配置

## 重要な前提

Arduino UNOはアナログ入力がA0-A5の6本で、A4/A5はMPU-6050のI2Cに使う。4個カラーセンサーと距離センサーを同時に読むにはアナログ入力が足りないため、74HC4051等のアナログマルチプレクサを使う前提にした。

マルチプレクサを使わない場合は、カラーセンサー数を減らす、距離センサーを別方式にする、Arduino Mega/Nano Every等へ変更する、外部ADCを追加する、のいずれかが必要。

## Arduinoピン

| Arduinoピン | 接続先 | 用途 |
|---|---|---|
| A0 | MUX共通出力 | カラー/距離センサーのアナログ読み取り |
| A1 | MUX S2 | デジタル出力としてチャンネル選択 |
| A4/SDA | MPU-6050 SDA | I2C |
| A5/SCL | MPU-6050 SCL | I2C |
| D2 | MUX S0 | チャンネル選択 |
| D4 | MUX S1 | チャンネル選択 |
| D3 | FL motor PWM | 前左モーターPWM |
| D7 | FL motor DIR | 前左モーター方向 |
| D5 | FR motor PWM | 前右モーターPWM |
| D8 | FR motor DIR | 前右モーター方向 |
| D6 | RL motor PWM | 後左モーターPWM |
| D12 | RL motor DIR | 後左モーター方向 |
| D11 | RR motor PWM | 後右モーターPWM |
| D13 | RR motor DIR | 後右モーター方向 |
| D10 | MG996R signal | サーボ制御 |

`Servo`ライブラリはUNOのTimer1を使うため、D9/D10のPWMはモーター用に使わない。

## MUXチャンネル

| MUXチャンネル | 接続先 |
|---|---|
| CH0 | カラーセンサー S1 |
| CH1 | カラーセンサー S2 |
| CH2 | カラーセンサー S3 |
| CH3 | カラーセンサー S4 |
| CH4 | GP2Y0A21YK距離センサー |
| CH5-CH7 | 未使用 |

## 電源

- Arduino: USBまたは安定した5V/VIN入力。
- サーボ: 4.8-6.6Vの別電源推奨。
- DCモーター: 6V系の別電源推奨。
- MPU/センサー: 実物基板仕様に合わせて3.3Vまたは5V。
- すべてのGNDは共通化する。

## モーター方向調整

横移動や旋回が意図と逆の場合は、配線を入れ替える前に`.ino`上部の以下を変更する。

```cpp
const int FL_DIRECTION_SIGN = 1;
const int FR_DIRECTION_SIGN = 1;
const int RL_DIRECTION_SIGN = 1;
const int RR_DIRECTION_SIGN = 1;
```

該当モーターだけ`-1`へ変更して確認する。
