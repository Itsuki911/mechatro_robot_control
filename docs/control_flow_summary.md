# 制御フローまとめ

## 1周期の処理順

1. カラーセンサー、超音波センサー、MPUを読む。
2. 白線位置、直線滞在時間、カーブ滞在時間を更新する。
3. 状態遷移を判定する。
4. 目標サーボ角とDCモーター速度を決める。
5. `MAX_SERVO_STEP` と `MAX_SPEED_STEP` で実出力を滑らかにする。
6. CSVログと1秒ごとの状態サマリーを出力する。

## 状態

| 状態 | 役割 |
|---|---|
| `STATE_INIT` | 起動直後 |
| `STATE_CALIBRATION` | センサー安定待ち |
| `STATE_LINE_TRACE` | 通常の白線追従 |
| `STATE_STRAIGHT_TRACE` | 中央センサーが白線上に長くいる直線 |
| `STATE_CURVE_ENTRY` | カーブ検知直後 |
| `STATE_CURVE_TRACE` | カーブ追従 |
| `STATE_LINE_RECOVERY` | 全センサー黒床/0相当からの低速直進復帰 |
| `STATE_BACKTRACK` | 復帰しない場合の短時間後退 |
| `STATE_OBSTACLE_DETECTED` | 障害物検知直後の停止 |
| `STATE_OBSTACLE_AVOIDANCE` | サーボを切った低速回避 |
| `STATE_TILT_RECOVERY` | roll角異常からの低速復帰 |
| `STATE_GOAL` | ゴール停止 |
| `STATE_EMERGENCY_STOP` | 危険時停止 |

## 白線/黒床判定

`LINE_IS_WHITE=true` のため、値が `WHITE_LINE_THRESHOLD` 以上なら白線、`BLACK_FLOOR_THRESHOLD` 以下なら黒床と扱います。全センサー黒床または全センサー0相当ならラインロスト候補です。

## ゴール判定

スタート直後は `GOAL_IGNORE_START_MS` の間ゴール判定を無効化します。その後、全センサーが白線を `GOAL_CONFIRM_MS` 以上検出した場合だけ `STATE_GOAL` へ遷移します。
