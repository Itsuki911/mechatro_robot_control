# C / C++制御ロジック

このディレクトリはArduino APIに依存しない、学習・確認用の制御判断サンプルです。実機で書き込む本体は `arduino/mechatro_robot_control/` です。

`SensorData` に白線/黒床センサー値、超音波距離、roll角を入れて `RobotController::update()` を呼ぶと、後輪DCモーターの `driveSpeed` と前輪サーボの `servoDeg` を持つ `MotorCommand` が返ります。

## ビルド例

```sh
cd cpp
g++ -std=c++17 -Wall -Wextra -pedantic main.cpp RobotController.cpp -o robot_controller_demo
./robot_controller_demo
```

## 注意

Arduino版と同じ考え方をPC上で追いやすくするための簡易モデルです。実機ピン、CSVログ、MPU/超音波の読み取りはArduino側のファイルを確認してください。
