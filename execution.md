# 実行手順

このファイルは、ロボットを動かすためにArduino側で行うこと、PC側で行うことを分けてまとめた手順書です。

## 0. 前提

- Arduino UNOを使う。
- カラーセンサー4つはRGBのG出力のみを使う。
- カラーセンサーはA0-A3へ直接接続する。MUXは使わない。
- MPU-6050はA4/A5のI2Cへ接続する。
- コースは白線/黒床。
- センサー配置はひし形。

```text
        進行方向
           ↑
           S1
        S2    S3
           S4
```

## 1. Arduino側で行うこと

### 1.1 配線確認

| 部品 | 接続 |
|---|---|
| カラーセンサーS1 G出力 | A0 |
| カラーセンサーS2 G出力 | A1 |
| カラーセンサーS3 G出力 | A2 |
| カラーセンサーS4 G出力 | A3 |
| MPU-6050 SDA | A4 |
| MPU-6050 SCL | A5 |
| 後輪DCモーター PWM | D5 |
| 後輪DCモーター DIR | D7 |
| 超音波 TRIG | D8 |
| 超音波 ECHO | D9 |
| サーボ信号 | D10 |

確認すること:

- Arduino、モーター電源、サーボ電源、センサー電源のGNDを共通化する。
- サーボとDCモーターをArduino 5Vピンから直接駆動しない。
- 最初のモーターテストではタイヤを浮かせる。

### 1.2 Arduino CLIでビルド

```sh
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno arduino/mechatro_robot_control
```

補助スクリプトを使う場合:

```sh
scripts/arduino_compile.sh
```

### 1.3 Arduinoへ書き込み

ポートを確認します。

```sh
arduino-cli board list
```

書き込みます。

```sh
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:uno arduino/mechatro_robot_control
```

補助スクリプトを使う場合:

```sh
scripts/arduino_upload.sh /dev/cu.usbmodemXXXX
```

### 1.4 単体テスト

安全のため、最初に以下の順でテストします。

1. `arduino/tests/color_sensor_test/color_sensor_test.ino`
   - S1-S4の白線値、黒床値を確認する。
2. `arduino/tests/servo_test/servo_test.ino`
   - サーボが中央、左、右に動くか確認する。
3. `arduino/tests/dc_motor_test/dc_motor_test.ino`
   - 後輪DCモーターが低速前進、停止、短時間後退するか確認する。
4. `arduino/tests/ultrasonic_test/ultrasonic_test.ino`
   - 10cm、20cm、30cmで距離値を確認する。
5. `arduino/tests/mpu_test/mpu_test.ino`
   - 水平、左傾き、右傾きでroll角が変わるか確認する。
6. `arduino/tests/integration_test/integration_test.ino`
   - タイヤを浮かせた状態でサーボとDCモーターを同時確認する。

### 1.5 本番スケッチの確認

本番スケッチを書き込んだら、シリアルモニターを開きます。

```sh
arduino-cli monitor -p /dev/cu.usbmodemXXXX -c baudrate=115200
```

確認するCSV列:

- `s1`, `s2`, `s3`, `s4`
- `state`
- `line_error`
- `straight_ms`
- `curve_ms`
- `servo_deg`
- `drive_speed`
- `estimated_distance_cm`
- `error_flags`

## 2. PC側で行うこと

### 2.1 Python環境を作る

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install -r python/requirements.txt
```

### 2.2 ダミーデータで解析コードを確認

実機を動かす前に、PCだけで解析コードを確認します。

```sh
python python/analyze_log.py data/dummy_sensor_clean.csv --no-plot
python python/analyze_log.py data/dummy_sensor_noisy.csv --no-plot
```

グラフも作る場合:

```sh
python python/analyze_log.py data/dummy_sensor_clean.csv
python python/analyze_log.py data/dummy_sensor_noisy.csv
```

### 2.3 実機ログを保存

ArduinoをUSB接続し、ポートを指定してログを保存します。

```sh
python python/serial_logger.py --port /dev/cu.usbmodemXXXX --baud 115200
```

ログは `logs/robot_log_YYYYMMDD_HHMMSS.csv` に保存されます。

### 2.4 実機ログを解析

```sh
python python/analyze_log.py logs/robot_log_YYYYMMDD_HHMMSS.csv
```

見るポイント:

- S1/S4が直線区間で白線を拾っているか。
- S2が左カーブ、S3が右カーブで反応しているか。
- 150mm途切れ区間で `LINE_RECOVERY` や `BACKTRACK` に入りすぎていないか。
- ゴール候補とゴール確定がログに出ているか。
- `error_flags` が増え続けていないか。

## 3. 初回走行の流れ

1. タイヤを浮かせて、全テストスケッチを確認する。
2. カラーセンサーの白線/黒床値を記録し、`Config.h` の閾値を調整する。
3. 本番スケッチをビルドして書き込む。
4. PCで `serial_logger.py` を起動する。
5. ロボットを低速設定のまま短距離だけ走らせる。
6. 保存CSVを `analyze_log.py` で解析する。
7. `STRAIGHT_SPEED`, `CURVE_SPEED`, `MAX_SERVO_STEP`, `GOAL_CONFIRM_MS` を少しずつ調整する。

## 4. 安全停止

異常が出た場合は、まず電源を切るかUSBシリアルから停止コマンドを送ります。現在の簡易コマンドでは、シリアルに `e` を送ると `stopActuators()` を呼びます。

```text
e
```

Emergency Stop後は、原因を確認してから再起動してください。
