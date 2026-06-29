# mechatro_robot_control

白線/黒床コースを走るArduino UNO向けロボット制御コードです。今回の主構成は、前輪サーボで操舵し、後輪DCモーターで前進/停止/短時間後退を行う実機です。センサーCSVログをPythonで保存・解析できるようにしています。

## 実機構成

- コース: 白線 / 黒床
- 操舵: 前輪サーボ
- 駆動: 後輪DCモーター
- ライン検知: カラーセンサー S1-S4のG出力(A0-A3直結、ひし形配置)
- 距離検知: デジタルピン接続の超音波センサー
- 姿勢検知: MPU-6050

古いメカナム4輪前提の説明は、今回の実機構成とは異なります。`cpp/` は制御ロジック学習用の旧構成サンプルとして残し、実機スケッチは `arduino/mechatro_robot_control/` を正とします。

## ファイル構成

```text
arduino/mechatro_robot_control/
  mechatro_robot_control.ino  setup/loop
  Config.h                    ピン、閾値、速度、時間定数
  Types.h                     状態、センサー、指令、エラー型
  Sensors.*                   カラー、超音波、MPU読み取り
  Controller.*                状態遷移、白線追従、復帰、ゴール判定
  Actuators.*                 サーボ、DCモーター、ランプ処理
  DebugSerial.*               CSVログと簡易コマンド
arduino/tests/                実機確認用の低速テストスケッチ
python/                       ログ収集、解析、ダミーデータ生成
data/                         clean/noisyの評価用CSV
docs/                         配線、調整、テスト、CLI手順
scripts/                      Arduino CLI補助スクリプト
```

## Arduino CLI

```sh
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno arduino/mechatro_robot_control
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:uno arduino/mechatro_robot_control
arduino-cli monitor -p /dev/cu.usbmodemXXXX -c baudrate=115200
```

補助スクリプト:

```sh
scripts/arduino_compile.sh
scripts/arduino_upload.sh /dev/cu.usbmodemXXXX
scripts/arduino_monitor.sh /dev/cu.usbmodemXXXX
```

詳しくは [docs/arduino_cli_setup.md](docs/arduino_cli_setup.md) を参照してください。

## Pythonログ収集と解析

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install -r python/requirements.txt
python python/serial_logger.py --port /dev/cu.usbmodemXXXX --baud 115200
python python/analyze_log.py data/dummy_sensor_clean.csv
python python/analyze_log.py data/dummy_sensor_noisy.csv
```

CSV列は `time_ms,state,s1,s2,s3,s4,line_pos,line_error,straight_ms,curve_ms,distance_cm,roll_deg,servo_deg,drive_speed,estimated_distance_cm,error_flags,...` です。

## 実機調整パラメータ

主な調整値は `arduino/mechatro_robot_control/Config.h` にあります。

| 定数 | 用途 |
|---|---|
| `WHITE_LINE_THRESHOLD` | 白線判定の下限 |
| `BLACK_FLOOR_THRESHOLD` | 黒床判定の上限 |
| `BASE_SPEED`, `STRAIGHT_SPEED`, `CURVE_SPEED` | 走行速度 |
| `MAX_SPEED_STEP`, `MAX_SERVO_STEP` | 急変化防止 |
| `STRAIGHT_CONFIRM_MS`, `CURVE_CONFIRM_MS` | 滞在時間による直線/カーブ判定 |
| `GOAL_CONFIRM_MS`, `GOAL_IGNORE_START_MS` | ゴール誤検知防止 |
| `ALL_FLOOR_CONFIRM_MS`, `LOST_LINE_FORWARD_MS`, `BACKTRACK_MS` | ラインロスト復帰 |
| `OBSTACLE_DISTANCE_CM` | 障害物検知距離 |

推定移動距離はエンコーダなしの概算です。`ESTIMATED_SPEED_CM_PER_SEC_AT_FULL_PWM` は実測で補正してください。

カラーセンサーはMUXを使わず、S1を前方、S2を左、S3を右、S4を後方に置くひし形配置です。MPU-6050はA4/A5のI2Cを使います。

## テストスケッチ

- `arduino/tests/color_sensor_test/color_sensor_test.ino`
- `arduino/tests/servo_test/servo_test.ino`
- `arduino/tests/dc_motor_test/dc_motor_test.ino`
- `arduino/tests/ultrasonic_test/ultrasonic_test.ino`
- `arduino/tests/mpu_test/mpu_test.ino`
- `arduino/tests/integration_test/integration_test.ino`

書き込み前はタイヤを浮かせ、モーター/サーボ別電源のGND共通を確認してください。Emergency Stopではモーター停止、サーボ中央に寄せます。
