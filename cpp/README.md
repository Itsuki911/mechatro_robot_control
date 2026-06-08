# C / C++制御ロジック

このディレクトリはArduino APIに依存しない制御判断部分です。`SensorData`にセンサー値を入れて`RobotController::update()`を呼ぶと、メカナム4輪の速度指令とサーボ角度が`MotorCommand`として返ります。

## ビルド例

```sh
g++ -std=c++17 -Wall -Wextra -pedantic main.cpp RobotController.cpp -o robot_controller_demo
./robot_controller_demo
```

## 移植方針

Webotsなどでは、シミュレーション側でラインセンサー、距離、姿勢角を`SensorData`へ変換してください。モーター出力は`MotorCommand`の4輪値を各ホイール速度へ対応させます。実機との差はセンサー値のスケールとモーターの正転方向だけに閉じ込める方針です。
