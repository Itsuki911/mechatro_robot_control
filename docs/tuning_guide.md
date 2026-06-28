# 調整ガイド

## 白線/黒床の閾値

1. `arduino/tests/color_sensor_test/color_sensor_test.ino` を書き込む。
2. S1-S4を白線上に置き、各値を記録する。
3. S1-S4を黒床上に置き、各値を記録する。
4. 白線値と黒床値の中間を `WHITE_LINE_THRESHOLD` と `BLACK_FLOOR_THRESHOLD` の候補にする。

照明、センサー高さ、床材で値は変わります。走行前に毎回数十秒ログを取り、Pythonで値の範囲を確認してください。

## 直線判定

S2/S3のどちらかが白線を `STRAIGHT_CONFIRM_MS` 以上検出し続けると `STATE_STRAIGHT_TRACE` になります。直線でふらつく場合は `STRAIGHT_SPEED` を下げ、`MAX_SPEED_STEP` を小さくします。

## カーブ判定

左側S1/S2、右側S3/S4の滞在時間が `CURVE_CONFIRM_MS` を超えるとカーブ扱いになります。反応が遅い場合は短くし、ノイズで誤検知する場合は長くします。サーボの急変化は `MAX_SERVO_STEP` で抑えます。

## ゴール判定

全センサーが白線を `GOAL_CONFIRM_MS` 以上検出したときだけゴール確定します。スタート直後の誤停止を避けるため、`GOAL_IGNORE_START_MS` が経過するまではゴール判定しません。

## ラインロスト

全センサーが黒床または0相当なら、まず低速直進し、復帰しなければ短時間バックします。その後、直前のライン誤差方向へ探索します。

## 推定距離

`estimated_distance_cm` はエンコーダなしの概算です。実測距離とログの差から `ESTIMATED_SPEED_CM_PER_SEC_AT_FULL_PWM` を補正してください。
