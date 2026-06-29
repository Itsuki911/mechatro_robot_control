# ダミーデータ

`dummy_sensor_clean.csv` はノイズが少ない評価用ログ、`dummy_sensor_noisy.csv` はカラーセンサー、超音波距離、MPU roll、サーボ角、駆動速度に揺れを入れた評価用ログです。

どちらもArduino CSVログと同じ列を持ち、`python/analyze_log.py` で解析できます。

カラーセンサー列は、ひし形配置の `s1=前方`, `s2=左`, `s3=右`, `s4=後方` を表します。直線区間ではS1/S4、カーブ区間ではS2またはS3が白線値になるようにしています。

再生成:

```sh
python python/generate_dummy_data.py
```
