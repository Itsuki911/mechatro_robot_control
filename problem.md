# 現在の課題

このファイルは、実装済みの白線/黒床ライントレースに対して、今後検討が必要な判断メカニズムをまとめるためのメモです。

## 1. 150mmの線が途切れている部分の判断メカニズム

`course_page.png` に相当するコース画像、現リポジトリでは `docs/assets/course_page5.png` から確認できる、約150mmの白線が途切れている区間をどう扱うかを考える必要があります。

現在の基本方針は、全センサーが黒床または0相当になった場合に、すぐ停止せず以下の順で復帰を試すことです。

1. 一定時間、低速で直進する。
2. その間にS1/S4またはS2/S3が白線を再検出したらライン追従へ戻る。
3. 再検出できなければ短時間バックする。
4. 直前のライン誤差方向へ低速探索する。

150mmの途切れ区間では、ラインを見失った瞬間にすぐカーブ探索へ入ると不安定になります。そのため、途切れ区間専用の判断として、次の条件を検討します。

| 判断材料 | 使い方 |
|---|---|
| 直前状態 | `STATE_STRAIGHT_TRACE` 中なら、途切れ区間の可能性が高い |
| 推定距離 | `estimated_distance_cm` で150mm前後の直進継続を許可する |
| 直前ライン誤差 | 誤差が小さい場合は直進優先、誤差が大きい場合は探索優先 |
| S1/S4再検出 | ひし形配置の中央軸が復帰したら通常追従へ戻る |
| タイムアウト | 一定時間または推定距離を超えたら通常のラインロスト復帰へ入る |

実装案:

```text
STATE_LINE_GAP_CROSSING を追加するか、
STATE_LINE_RECOVERY の前半を「途切れ区間直進」として扱う。

条件:
- 直前が STRAIGHT_TRACE または LINE_TRACE
- allFloor が一定時間続いた
- lastLineError が小さい

動作:
- サーボ中央
- 低速直進
- 推定距離15cm相当まで白線再検出を待つ
- 再検出できれば LINE_TRACE
- できなければ BACKTRACK または LINE_RECOVERY
```

現時点ではエンコーダがないため、距離は推定値です。実機では、`ESTIMATED_SPEED_CM_PER_SEC_AT_FULL_PWM` を実測補正してから判断距離に使う必要があります。

## 2. goalの判断メカニズム

現在のゴール判定は、全センサーが白線を一定時間検出することを基本にしています。

```text
allLine が GOAL_CONFIRM_MS 続く
かつ
GOAL_IGNORE_START_MS 経過後
```

しかし、コース途中の横線、交差、途切れ区間からの復帰時に、全センサー白線に近い状態が一瞬出る可能性があります。誤停止を避けるため、次の条件を組み合わせる必要があります。

| 判断材料 | 目的 |
|---|---|
| `GOAL_IGNORE_START_MS` | スタート直後の誤判定を防ぐ |
| 全センサー白線の継続時間 | 横線の一瞬検知を防ぐ |
| 推定走行距離 | コース序盤や中盤の横線で止まらない |
| 直前状態 | `LINE_TRACE` または `STRAIGHT_TRACE` からのゴール候補だけ許可する |
| 速度 | ゴール候補時は徐々に減速し、確定後に停止する |
| ログ | ゴール候補開始時刻と確定時刻を記録する |

実装案:

```text
goal_candidate = allLine && elapsed_from_start > GOAL_IGNORE_START_MS

さらに追加候補:
- estimated_distance_cm > GOAL_MIN_DISTANCE_CM
- currentState が LINE_TRACE / STRAIGHT_TRACE / CURVE_TRACE のいずれか
- allLine が GOAL_CONFIRM_MS 継続
```

今回のロボットでは、ひし形配置のため全センサー白線はかなり強いパターンです。ただしライン幅、センサー間隔、横線幅によってはゴール以外でも発生し得るため、実機ログで以下を確認します。

1. コース途中の横線で `allLine` が何ms続くか。
2. ゴール地点で `allLine` が何ms続くか。
3. `GOAL_CONFIRM_MS` をどの値にすると誤判定しないか。
4. 推定距離を条件に入れる必要があるか。

## 今後の確認項目

- 150mm途切れ区間を通過した時の `all_zero`, `allFloor`, `estimated_distance_cm`, `line_error` をログで確認する。
- ゴール候補時に `event_code=EVENT_GOAL_CANDIDATE` が出るか確認する。
- 横線とゴールのログを比較し、`GOAL_CONFIRM_MS` と必要なら `GOAL_MIN_DISTANCE_CM` を決める。
- ひし形配置でS1/S4が直線判定に十分使えるか確認する。
