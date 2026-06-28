# mechatro_robot_control 学習ノート

このノートは、`arduino/mechatro_robot_control/` のArduinoスケッチを読みながら、入力、判断、出力の流れを理解するためのものです。

## 全体像

Arduinoの実行入口は `mechatro_robot_control.ino` です。

```cpp
void setup() {
  // 電源投入時に1回だけ実行
}

void loop() {
  // 電源が切れるまで繰り返し実行
}
```

今回の `loop()` は、約20msごとに次の順番で動きます。

1. `readSensors()` でカラーセンサー、超音波センサー、MPUを読む。
2. `updateController()` で白線位置、状態、目標サーボ角、駆動速度を決める。
3. `applyMotorCommand()` でサーボと後輪DCモーターへ出力する。
4. `printCsvLog()` でPythonが読めるCSVを出す。

## ファイルの役割

| ファイル | 役割 |
|---|---|
| `Config.h` | ピン、閾値、速度、時間定数をまとめる |
| `Types.h` | 状態、センサー値、モーター指令、エラーを定義する |
| `Sensors.cpp` | センサーを読む |
| `Controller.cpp` | 状態遷移と白線追従の判断をする |
| `Actuators.cpp` | サーボとDCモーターを実際に動かす |
| `DebugSerial.cpp` | CSVログと簡易コマンドを扱う |

## 白線/黒床

今回のコースは白線/黒床です。`Config.h` の `LINE_IS_WHITE=true` がその前提を表します。

- センサー値が `WHITE_LINE_THRESHOLD` 以上なら白線
- センサー値が `BLACK_FLOOR_THRESHOLD` 以下なら黒床
- 閾値の間は不明

S2/S3が長く白線上にいれば直線、S1/S2またはS3/S4が長く白線上にいればカーブと判断します。

## 状態機械

ロボットの行動は `RobotState` で管理します。状態を分けると、「今なぜこの動きをしているか」をログで追いやすくなります。

例:

- `STATE_LINE_TRACE`: 普通に白線追従
- `STATE_STRAIGHT_TRACE`: 直線と判断して少し速く走る
- `STATE_CURVE_TRACE`: カーブなので遅く走り、サーボ角を大きくする
- `STATE_LINE_RECOVERY`: 白線を見失ったので低速で探す
- `STATE_BACKTRACK`: 復帰できないので短時間バックする
- `STATE_GOAL`: ゴール判定後に停止

## 安全寄りの実装

実機破損を避けるため、サーボ角とモーター速度は一気に変えません。

- `MAX_SERVO_STEP`: 1周期で変えてよいサーボ角
- `MAX_SPEED_STEP`: 1周期で変えてよい速度

Emergency StopやGoalでは、モーターを停止し、サーボを中央に寄せます。

## Pythonログ

ArduinoはCSV形式でログを出します。Arduino IDEのシリアルモニターでも見られますが、Pythonで保存すると後からグラフ化できます。

```sh
python python/serial_logger.py --port /dev/cu.usbmodemXXXX --baud 115200
python python/analyze_log.py data/dummy_sensor_clean.csv
```

最初は実機ではなく `data/dummy_sensor_clean.csv` と `data/dummy_sensor_noisy.csv` を解析し、列の意味を確認してください。
