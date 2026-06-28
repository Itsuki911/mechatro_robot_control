# ダミーデータ

`dummy_sensor_clean.csv` はノイズが少ない評価用ログ、`dummy_sensor_noisy.csv` はカラーセンサー、超音波距離、MPU roll、サーボ角、駆動速度に揺れを入れた評価用ログです。

どちらもArduino CSVログと同じ列を持ち、`python/analyze_log.py` で解析できます。

再生成:

```sh
python python/generate_dummy_data.py
```
