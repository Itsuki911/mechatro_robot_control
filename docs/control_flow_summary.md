# 制御フローまとめ

## M_flowchart.pngから読み取った流れ

1. START
2. センサー初期化
   - Arduino、各センサー、サーボ、DCモーター、閾値を初期化
   - カラーセンサーをキャリブレーション
3. メイン制御ループ
4. センサー読み取り
   - S1, S2, S3, S4
   - ToF/距離センサー
   - MPU roll angle
5. roll角が大きい場合
   - TILT RECOVERY
   - 速度低下、操舵角補正
6. 障害物検知
   - OBSTACLE AVOIDANCE
   - 速度低下、サーボを切る、回避動作
7. ラインが見つからない場合
   - LINE RECOVERY
   - 黒線を探索
8. ラインパターン判定
   - 全白: ライン復帰へ
   - S1黒かつS2/S3黒: 側線を無視して直進
   - S2/S3黒かつS1白: カーブ進入
   - S2のみ黒: 左補正
   - S3のみ黒: 右補正
9. カーブ追従
   - S1黒&S3黒: やや右
   - S1黒&S2黒: やや左
   - S1&S4黒、S2&S3白: カーブ完了、通常復帰
   - S1白&S2黒: 強く左
   - S1白&S3黒: 強く右
10. 通常ライン追従
    - S1&S4黒: 直進、高め速度
    - S2黒: やや左補正
    - S3黒: やや右補正
11. ゴール検知
    - DCモーター停止
    - サーボ停止/中央

## 実装した状態

| 状態 | 役割 | 主な遷移 |
|---|---|---|
| `STATE_INIT` / `Init` | 起動直後 | キャリブレーションへ |
| `STATE_CALIBRATION` / `Calibration` | センサー安定待ち | 一定時間後ライン追従へ |
| `STATE_LINE_TRACE` / `LineTrace` | 通常の黒線追従 | カーブ、障害物、傾き、ゴールへ |
| `STATE_CURVE_ENTRY` / `CurveEntry` | 外側センサー反応時のカーブ進入 | カーブ追従へ |
| `STATE_CURVE_TRACE` / `CurveTrace` | 強めの補正で曲線追従 | 通常追従またはライン復帰へ |
| `STATE_OBSTACLE_DETECTED` / `ObstacleDetected` | 障害物検知直後の一時停止 | 回避へ |
| `STATE_OBSTACLE_AVOIDANCE` / `ObstacleAvoidance` | 横移動/旋回で障害物回避 | ライン復帰へ |
| `STATE_LINE_RECOVERY` / `LineRecovery` | 全白やライン外れ時の探索 | 黒線検出で追従へ |
| `STATE_TILT_RECOVERY` / `TiltRecovery` | 軽い傾きの補正 | 傾き低下で追従へ |
| `STATE_GOAL` / `Goal` | ゴール停止 | 停止維持 |
| `STATE_EMERGENCY_STOP` / `EmergencyStop` | 大きな傾きや異常 | 停止維持 |

## 不明瞭だった点と解釈

- フローチャートの`S1 & S4 both black`は、コース上の横線またはカーブ完了の目印と解釈した。
- ゴール判定は4センサー同時黒が一定時間続くことにした。コース途中にも交差線があるため、単発検知では止めない。
- 障害物回避は詳細な軌道指定が読めなかったため、メカナムの右横移動+微前進を初期値にした。
- 元資料はバイク型、課題はメカナム指定なので、コードはメカナム優先。サーボは補助操舵として残した。
