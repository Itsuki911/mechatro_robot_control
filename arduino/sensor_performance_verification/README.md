# センサー・アクチュエータ性能検証スケッチ

このフォルダは、ロボット制御本体やPython評価環境から独立した、Arduino IDE用の単体性能検証コードです。

Arduino IDEでは、各 `.ino` を同名フォルダ内の単独スケッチとして開いてください。

## 商品URL・データシート

### カラーセンサー

- 商品ページ: https://akizukidenshi.com/catalog/g/g109763/
- データシート: https://akizukidenshi.com/goodsaffix/s9032-02.pdf

### 距離モジュール

- 商品ページ: https://akizukidenshi.com/catalog/g/g102551/
- データシート: https://akizukidenshi.com/goodsaffix/gp2y0a21yk_e.pdf

### サーボモータ

- 商品ページ: https://akizukidenshi.com/catalog/g/g112534/
- データシート: https://akizukidenshi.com/goodsaffix/mg996r.pdf

### DCモータ

- 商品ページ: https://akizukidenshi.com/catalog/g/g115144/
- データシート: https://akizukidenshi.com/goodsaffix/MG1012-0635-298L.pdf

## スケッチ一覧

- `color_sensor_g_output_check/color_sensor_g_output_check.ino`
  - 4個のカラーセンサーG出力をA0-A3で読み、白線判定の0/1と生値をCSV出力します。
- `digital_distance_signal_check/digital_distance_signal_check.ino`
  - デジタル距離検知信号をD2で読み、検知/非検知をCSV出力します。GP2Y0A21YK単体はアナログ出力なので、デジタル入力で使う場合は比較器などでHIGH/LOWへ変換してください。
- `servo_mg996r_check/servo_mg996r_check.ino`
  - MG996Rサーボを安全寄りの角度範囲でゆっくり動かします。
- `dc_motor_mg1012_check/dc_motor_mg1012_check.ino`
  - DCモータを低PWMから段階的に回し、停止と方向確認を行います。

## 共通注意

- 書き込み前に配線、電源、GND共通を確認してください。
- モータやサーボを動かす検証では、タイヤを浮かせてください。
- サーボとDCモータはArduino 5Vから直接大電流を取らず、外部電源を使い、GNDをArduinoと共通にしてください。
- シリアルモニターは115200bpsに設定してください。
