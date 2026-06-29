# ダミーデータ

`dummy_sensor_clean.csv` はノイズが少ない評価用ログ、`dummy_sensor_noisy.csv` はカラーセンサー、超音波距離、MPU roll、サーボ角、駆動速度に揺れを入れた評価用ログです。

どちらもArduino CSVログと同じ列を持ち、`python/analyze_log.py` で解析できます。

カラーセンサー列は、ひし形配置の `s1=前方`, `s2=左`, `s3=右`, `s4=後方` を表します。直線区間ではS1/S4、カーブ区間ではS2またはS3が白線値になるようにしています。

## course_page5_sensor_*.csv

`course_page5_sensor_clean.csv` と `course_page5_sensor_noisy.csv` は、`docs/assets/course_page5.png` の寸法に寄せた長めの評価データです。フィールド寸法、線幅20mm、R300の中央ループ、右側の大きな曲線、150mmのライン切れ、3か所の障害物を反映しています。

追加列:

- `estimated_distance_mm`: Arduino側の `ESTIMATED_SPEED_CM_PER_SEC_AT_FULL_PWM=28.0` を 280mm/s として、`drive_speed` と `time_ms` から積算した推定走行距離です。
- `course_distance_mm`: 後退時だけ減算する、コース上の概算進行距離です。
- `motor_pwm`: `drive_speed` をPWM目安へ変換した値です。
- `s1_line`, `s2_line`, `s3_line`, `s4_line`: 各カラーセンサーが白線上なら1、黒床なら0です。
- `ultrasonic_detected`: 障害物が超音波検知距離に入った区間だけ1です。`ultrasonic_ok` はセンサー通信が正常かを示す既存列なので、通常は1のままです。
- `course_segment`: コース上の想定区間名です。

コース用データの目安:

- 約151.5秒
- 推定走行距離は約11080mm
- 150mmの線切れ区間では `s1_line` から `s4_line` がすべて0になります。
- 障害物の停止・回避区間では `ultrasonic_detected` が1になり、`distance_cm` が18cm以下になります。

再生成:

```sh
python python/generate_dummy_data.py
```
