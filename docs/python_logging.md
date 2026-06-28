# Pythonログ収集と解析

## 環境構築

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install -r python/requirements.txt
```

## ログ保存

```sh
python python/serial_logger.py --port /dev/cu.usbmodemXXXX --baud 115200
```

ログは `logs/robot_log_YYYYMMDD_HHMMSS.csv` に保存されます。`logs/` の実測CSVはGit管理しません。

## 解析

```sh
python python/analyze_log.py logs/robot_log_YYYYMMDD_HHMMSS.csv
python python/analyze_log.py data/dummy_sensor_clean.csv
python python/analyze_log.py data/dummy_sensor_noisy.csv
```

解析では状態遷移の件数、直線/カーブ滞在時間、推定距離、エラーフラグ回数を表示し、グラフを `logs/analysis/` に保存します。

## CSVの見方

- `s1`-`s4`: カラーセンサー生値
- `line_pos`, `line_error`: 白線位置と左右誤差
- `straight_ms`, `curve_ms`: 対象センサーが白線上にいた継続時間
- `distance_cm`: 超音波センサー距離
- `servo_deg`, `drive_speed`: 実出力に近い操舵角と駆動指令
- `estimated_distance_cm`: 時間と速度からの概算距離
- `error_flags`: 0以外なら異常あり

## トラブルシューティング

- CSVが保存されない: Arduino側のボーレートが115200か確認する。
- 文字化けする: `--baud` とシリアルモニター設定を揃える。
- グラフが出ない: `pip install -r python/requirements.txt` を再実行する。
- `Permission denied`: macOSのポート名が `/dev/cu.*` か確認する。
