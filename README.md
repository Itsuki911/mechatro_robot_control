# mechatro_robot_control

授業課題コースを安定して走行するための、Arduino実機用コードとC/C++制御ロジックの初期実装です。完璧な一発完成ではなく、実機テストで閾値・速度・ピン・モーター方向を調整しながら改善する前提で作っています。

## 構成

```text
mechatro_robot_control/
├── arduino/
│   └── mechatro_robot_control.ino
├── cpp/
│   ├── main.cpp
│   ├── RobotController.h
│   ├── RobotController.cpp
│   ├── SensorData.h
│   ├── MotorCommand.h
│   └── README.md
├── docs/
│   ├── hardware_summary.md
│   ├── control_flow_summary.md
│   ├── course_strategy.md
│   ├── pin_assignment.md
│   └── assets/
└── README.md
```

## 参考にした資料

- `~/Downloads/robot_architecture.pptx.pdf`
- `~/Downloads/M_flowchart.png`
- `~/Downloads/mechatro1_week4_mon (1).pdf` 5ページ目
- 指定された部品リンクの商品情報

指定名の`robot_arcitecture.pptx.pdf`は見つからず、近い名前の`robot_architecture.pptx.pdf`を確認しました。Notionの「メカトロ_M2」は、このセッションで検索ツールが公開されていなかったため未確認です。

## 使用部品

- Arduino UNO互換機
- MPU-6050
- RGBカラーセンサー S9032-02 x4
- Sharp GP2Y0A21YK距離センサー
- MG996Rサーボ
- DCギアードモーター x4
- DRV8835モータードライバー x2
- メカナムホイール x4
- タミヤ ボールキャスター
- バッテリー
- 追加前提: 74HC4051相当のアナログマルチプレクサ

## 制御方針

4個のラインセンサーS1-S4で黒/白/その他を判定し、通常はS2/S3を中心に追従します。S1/S4が反応したときはカーブや横線として扱い、速度を落として補正を強めます。距離センサーで障害物を見つけたら停止後に低速横移動し、ラインを失ったら直前の誤差方向へ探索します。MPUのroll角が大きい場合は減速補正し、危険角度では緊急停止します。

## 状態遷移

`Init`、`Calibration`、`LineTrace`、`CurveEntry`、`CurveTrace`、`ObstacleDetected`、`ObstacleAvoidance`、`LineRecovery`、`TiltRecovery`、`Goal`、`EmergencyStop`を実装しています。詳細は[control_flow_summary.md](/Users/adachiitsuki/Desktop/mechatro_robot_control/docs/control_flow_summary.md)を参照してください。

## ピン配置

ピン配置は[pin_assignment.md](/Users/adachiitsuki/Desktop/mechatro_robot_control/docs/pin_assignment.md)にまとめています。UNOの入力数制約により、カラーセンサーと距離センサーはマルチプレクサ経由で読む前提です。

## Arduino IDEでの書き込み

1. Arduino IDEで[mechatro_robot_control.ino](/Users/adachiitsuki/Desktop/mechatro_robot_control/arduino/mechatro_robot_control.ino)を開く。
2. ボードをArduino UNOまたは互換ボードに設定する。
3. 標準ライブラリ`Wire`と`Servo`が使えることを確認する。
4. 書き込み前に、モーターを浮かせた状態で電源とGND共通化を確認する。
5. Serial Monitorを115200bpsで開き、S1-S4、距離、roll角を確認する。
6. `BLACK_THRESHOLD`、`WHITE_THRESHOLD`、速度、モーター方向を調整する。

## C/C++版の実行

```sh
cd /Users/adachiitsuki/Desktop/mechatro_robot_control/cpp
g++ -std=c++17 -Wall -Wextra -pedantic main.cpp RobotController.cpp -o robot_controller_demo
./robot_controller_demo
```

## 実機で調整が必要なパラメータ

| パラメータ | 目的 |
|---|---|
| `BLACK_THRESHOLD` | 黒ライン判定 |
| `WHITE_THRESHOLD` | 白床判定 |
| `BASE_SPEED` | 通常速度 |
| `CURVE_SPEED` | カーブ速度 |
| `RECOVERY_SPEED` | ライン復帰/回避速度 |
| `MAX_SPEED_STEP` | 急加速防止 |
| `OBSTACLE_DISTANCE_CM` | 障害物判定距離 |
| `TILT_RECOVER_DEG` | 傾き補正開始角 |
| `TILT_STOP_DEG` | 緊急停止角 |
| `GOAL_CONFIRM_MS` | ゴール誤検知防止時間 |
| `OBSTACLE_AVOID_MS` | 回避動作時間 |
| `*_DIRECTION_SIGN` | モーター回転方向 |

## Webotsなどへの移植

シミュレーション側でラインセンサー、距離、roll角を`SensorData`へ変換し、`RobotController::update()`の戻り値を各ホイール速度へ割り当てます。Arduino固有処理は`.ino`側へ寄せているため、制御判断は`cpp/`側をベースに移植できます。
