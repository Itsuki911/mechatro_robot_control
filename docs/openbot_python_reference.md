# OpenBot Python実装の参考メモ

調査対象: [ob-f/OpenBot `python/`](https://github.com/ob-f/OpenBot/tree/master/python)

このメモは、OpenBotのArduino firmwareとPython実装から、`mechatro_robot_control` に参考になりそうな設計を抜き出したものです。OpenBotはスマートフォン制御が主系統ですが、`python/` にはLinux PCからカメラ、ジョイスティック、AI推論、Arduinoシリアル制御を扱う代替実装があります。

## 見たファイル

- [python/README.md](https://github.com/ob-f/OpenBot/blob/master/python/README.md)
- [python/run.py](https://github.com/ob-f/OpenBot/blob/master/python/run.py)
- [python/infer.py](https://github.com/ob-f/OpenBot/blob/master/python/infer.py)
- [python/joystick.py](https://github.com/ob-f/OpenBot/blob/master/python/joystick.py)
- [python/generate_data_for_training.py](https://github.com/ob-f/OpenBot/blob/master/python/generate_data_for_training.py)
- [python/tests/test_motor.py](https://github.com/ob-f/OpenBot/blob/master/python/tests/test_motor.py)
- [firmware/README.md](https://github.com/ob-f/OpenBot/blob/master/firmware/README.md)

## ArduinoとPythonの役割分担

OpenBotでは、Arduino側とPython側の責務がかなりはっきり分かれています。

| 層 | OpenBotでの役割 | このプロジェクトへの読み替え |
|---|---|---|
| Arduino firmware | モーターPWM、LED、超音波、電圧、エンコーダなどの低レベルI/O | サーボ角、後輪DCモーター、カラーセンサー、超音波、MPU、CSVログ |
| Python | カメラ入力、AI推論、ジョイスティック入力、ログ保存、テスト | シリアルログ保存、グラフ化、ダミーデータ解析、将来的な自動評価 |
| シリアル通信 | Python/上位アプリからArduinoへコマンドを送り、Arduinoから状態を返す | Arduino CSVログをPythonで読む。将来的にはPythonからテストコマンドを送る |

OpenBotのfirmware READMEでは、MCUは車体と上位アプリの橋渡しとして、シリアルで制御信号を受け取り、PWMへ変換し、センサー値を返す設計になっています。この分担は、今回のArduinoスケッチでも参考になります。つまり、Arduinoは実時間の安全制御とセンサー読み取りを担い、Pythonは記録、評価、可視化、テスト自動化を担う形が自然です。

## シリアル通信の考え方

OpenBot firmwareは、Serial Monitorや上位側から以下のような短いコマンドを受け取ります。

| コマンド例 | 意味 |
|---|---|
| `c<left>,<right>` | 左右モーター指令。例: `c255,255` で前進、`c0,0` で停止 |
| `s<time_ms>` | 超音波測距の送信周期設定 |
| `w<time_ms>` | ホイール回転数/オドメトリ送信周期設定 |
| `v<time_ms>` | 電圧測定送信周期設定 |
| `f` | ロボット種別や有効機能の問い合わせ |

Python側の最小モーターテストは [python/tests/test_motor.py](https://github.com/ob-f/OpenBot/blob/master/python/tests/test_motor.py) にあり、`serial.Serial("/dev/ttyUSB0", baudrate=115200)` でArduinoへ接続し、`c255,255\n` を送って5秒後に `c0,0\n` で止める構成です。

今回のプロジェクトでは、すでにArduinoからCSVログをPythonで読む方向を実装しています。OpenBotを参考にするなら、次の段階としてPythonからArduinoへ安全なテストコマンドを送る仕組みを追加できます。

例:

```text
T,servo,90
T,drive,0.15
T,stop
T,log_header
```

ただし実機破損を避けるため、OpenBotの `c255,255` のような高出力コマンドではなく、低速・短時間・タイムアウト付きのテストコマンドに限定するのがよいです。

## Python側の実行モード

OpenBotの [python/README.md](https://github.com/ob-f/OpenBot/blob/master/python/README.md) では、`run.py` に3つのモードがあります。

| モード | 内容 | 参考になる点 |
|---|---|---|
| Debug | 実機カメラやジョイスティックではなく、保存済みデータを使ってオフライン実行 | 今回の `data/dummy_sensor_clean.csv` / `dummy_sensor_noisy.csv` と同じ考え方 |
| Inference | 実カメラ画像とジョイスティック指示を使ってAI推論し、モーターへ出力 | 将来的に画像認識や自動判断を追加する場合の参考 |
| Joystick | ジョイスティックで手動操作し、学習用データも収集 | 実機調整時の手動テストUIとして参考 |

今回のプロジェクトでは、現時点ではAI推論よりも「ログ収集」「閾値調整」「状態遷移確認」が重要です。そのため、OpenBotのDebugモードの考え方が最も近いです。実機がなくても保存済みログで解析コードを動かせる設計は、すでに入れているダミーデータ解析と同じ方向です。

## `run.py` の役割

OpenBotの [python/run.py](https://github.com/ob-f/OpenBot/blob/master/python/run.py) は、Python側の入口です。引数で以下を切り替えます。

- `--policy_path`: 推論モデルの場所
- `--dataset_path`: Debugモードで使う保存済みデータ
- `--log_path`: 実行ログの保存先
- `--inference_backend`: `tf`, `tflite`, `openvino`
- `--run_mode`: `debug`, `inference`, `joystick`
- `--control_mode`: `dual`, `joystick`

今回のプロジェクトに置き換えるなら、将来的に `python/run_evaluation.py` のような入口を作り、以下を引数で切り替えられると扱いやすくなります。

- `--input`: 実機ログCSVまたはダミーデータCSV
- `--mode`: `analyze`, `live`, `serial-test`
- `--port`: Arduinoのシリアルポート
- `--out-dir`: 解析結果やログの保存先
- `--profile`: 白線/黒床の閾値設定セット

## `infer.py` で参考になる点

[python/infer.py](https://github.com/ob-f/OpenBot/blob/master/python/infer.py) では、次の点が参考になります。

1. `get_arduino_serial()` でArduino接続を関数化している。
2. `apply_action_serial()` でモーター指令をシリアルコマンドに変換している。
3. `control_frequency` を持ち、一定周期で制御する。
4. `debug` モードでは実機ではなく保存済みデータを使う。
5. `store_state()` と `write_log()` で、入力、出力、画像、コマンド履歴を後から学習・解析できる形で保存している。

今回のプロジェクトで特に参考になるのは、`apply_action_serial()` のように「Python内の抽象的な指令」と「Arduinoへ送る文字列プロトコル」を分けている点です。現在の `serial_logger.py` は受信中心ですが、送信を追加するなら次のように分けると読みやすくなります。

```python
def build_servo_test_command(deg: int) -> str:
    return f"T,servo,{deg}\n"

def send_command(serial_port, command: str) -> None:
    serial_port.write(command.encode("utf-8"))
```

## `joystick.py` で参考になる点

[python/joystick.py](https://github.com/ob-f/OpenBot/blob/master/python/joystick.py) は、pygameでジョイスティック入力を別スレッドとして読み、コールバックで制御側へ渡しています。

今回のロボットではすぐにジョイスティック操作は不要ですが、実機調整で以下のような用途に使えます。

- サーボ角を手動で少しずつ動かす。
- DCモーターを低速で前進/停止させる。
- センサー値を見ながら手動でライン復帰動作を試す。
- 緊急停止ボタンを割り当てる。

導入する場合は、最初から複雑な手動走行UIにせず、`stop`, `servo left/center/right`, `drive slow`, `drive stop` のような限定コマンドだけにするのが安全です。

## ログと学習データ生成

OpenBotの [generate_data_for_training.py](https://github.com/ob-f/OpenBot/blob/master/python/generate_data_for_training.py) は、実行ログを学習用データセット形式へ変換します。画像、制御値、コマンド履歴、姿勢などをフォルダへ分けて保存します。

今回のプロジェクトでは画像学習までは不要ですが、考え方は使えます。

| OpenBot | 今回のプロジェクト |
|---|---|
| `ctrlLog.txt` | `drive_speed`, `servo_deg` |
| `rgbFrames.txt` | 今は不要。将来カメラを使うなら追加 |
| `indicatorLog.txt` | `event_code`, 状態遷移ログ |
| `poseData.txt` | `roll_deg`, 推定距離 |
| pickleログ | CSVログで十分。必要なら後でParquet等へ拡張 |

今回の `python/analyze_log.py` はCSVを直接読む構成なので、当面はOpenBotほど複雑なデータセット変換は不要です。ただし、実機走行ログが増えたら `logs/` から `data/experiments/YYYYMMDD_*` のように整理する設計は参考になります。

## pytestによる検証

OpenBotのPython READMEでは、`tests/` 配下に以下のようなテストがあると説明されています。

- 推論テスト
- OpenVino変換テスト
- データ生成テスト
- ジョイスティック接続テスト
- Arduinoとのモーター接続テスト
- Realsenseカメラテスト

今回のプロジェクトでは、次のようなpytestを追加すると近い構成になります。

```text
python/tests/
  test_analyze_log.py          ダミーデータCSVを読み、必要列と集計が崩れないか確認
  test_generate_dummy_data.py  clean/noisy CSVを再生成して列を確認
  test_serial_protocol.py      Arduinoへ送る予定のテストコマンド文字列を確認
```

実機が必要なテストは、通常の自動テストからは外し、`--port` が指定された時だけ走らせるのが安全です。

## 今回のプロジェクトへ取り入れるとよい点

優先度順に並べると、次の通りです。

1. Python側に「受信ログ」だけでなく「安全なテストコマンド送信」を追加する。
2. Arduino側の `handleDebugCommands()` を少し拡張し、サーボテスト、低速モーターテスト、ログヘッダー再送信を受けられるようにする。
3. `python/tests/` を追加し、ダミーデータ解析とCSV列チェックをpytest化する。
4. 実機テストはOpenBotの `test_motor.py` のように短く保ち、必ず最後に停止コマンドを送る。
5. ログ保存フォルダを実験単位で整理し、解析結果PNGやメモを同じ実験フォルダへ残す。

## 取り入れない方がよい点

OpenBotはAI推論、カメラ、ジョイスティック、OpenVinoまで含むため、今回の白線/黒床ライントレースには重い部分もあります。

- AI推論やOpenVino対応は今すぐ不要。
- ジョイスティック常時制御は安全設計ができてからでよい。
- pickle中心のログ保存は、初学者にはCSVより追いにくい。
- Pythonから直接走行制御を常時行うより、Arduino側に安全停止と低レベル制御を残す方がよい。

## まとめ

OpenBotから一番参考になるのは、Arduinoを低レベル制御担当、Pythonを検証・入力・ログ・テスト担当に分ける構成です。今回の `mechatro_robot_control` はすでにArduinoからCSVを出し、Pythonで保存・解析する流れがあります。次の改善としては、OpenBotの `test_motor.py` や `apply_action_serial()` を参考に、PythonからArduinoへ安全な短時間テストコマンドを送る仕組みと、pytestによるダミーデータ検証を追加するのが現実的です。
