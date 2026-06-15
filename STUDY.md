# mechatro_robot_control 学習ノート

この資料は、`mechatro_robot_control.ino`を読みながらArduinoとC++の基礎を学ぶためのものです。
コードを変更する前に、まず「入力 -> 判断 -> 出力」という全体の流れをつかんでください。

## 1. プログラム全体の流れ

Arduinoのスケッチには、特別な関数が2つあります。

```cpp
void setup() {
  // 電源投入時またはリセット時に1回だけ実行される
}

void loop() {
  // setup()の後、電源が切れるまで繰り返し実行される
}
```

このプログラムの`loop()`は、約20ミリ秒ごとに次の処理を繰り返します。

1. `readSensors()`でライン、距離、傾きを読む。
2. `updateController()`でロボットの次の動きを決める。
3. `applyMotorCommand()`でサーボとモーターを動かす。
4. `printDebug()`で確認用データをシリアルモニターへ表示する。

## 2. `#include`とライブラリ

### `#include <Servo.h>`

`Servo`クラスを使えるようにします。サーボモーターへ角度を指示するために使用します。

主な処理:

- `Servo steeringServo;`: サーボを操作するオブジェクトを作る。
- `steeringServo.attach(pin);`: サーボ信号線のピンを登録する。
- `steeringServo.write(angle);`: 目標角度を0から180度で指定する。

### `#include <Wire.h>`

ArduinoのI2C通信機能を使えるようにします。このプログラムではMPU-6050との通信に使用します。

主な処理:

- `Wire.begin()`: ArduinoをI2C通信のコントローラーとして開始する。
- `Wire.beginTransmission(address)`: 指定したアドレスの機器への送信を始める。
- `Wire.write(value)`: レジスタ番号や設定値を送信バッファへ入れる。
- `Wire.endTransmission()`: 送信を実行して終了する。
- `Wire.requestFrom(address, count)`: 機器から指定バイト数を要求する。
- `Wire.available()`: 読み取れるデータ数を調べる。
- `Wire.read()`: 受信データを1バイト読む。

## 3. Arduinoで使う基本関数

### `pinMode(pin, mode)`

ピンを入力または出力として設定します。

- `OUTPUT`: Arduinoから電圧を出すピンとして使う。
- `INPUT`: 外部の電圧を読むピンとして使う。
- `INPUT_PULLUP`: 内蔵プルアップ抵抗付きの入力として使う。

### `digitalWrite(pin, value)`

デジタル出力を切り替えます。

- `HIGH`: UNOでは基本的に約5 Vの状態。
- `LOW`: 基本的に0 Vの状態。

このプログラムでは、マルチプレクサのチャンネル選択とモーター回転方向の指定に使います。

### `analogRead(pin)`

アナログ入力電圧を数値として読みます。標準設定のArduino UNOでは通常0から1023です。

- 0はおよそ0 V
- 1023は基準電圧付近

### `analogWrite(pin, value)`

対応ピンへPWM信号を出します。UNOでは通常0から255を指定します。

- 0: 常にOFF
- 255: 常にON
- その中間: ONとOFFの時間比を変えて平均的な出力を調整

名前に`analog`とありますが、UNOでは真の連続アナログ電圧ではなくPWMです。

### `millis()`

プログラム開始後の経過時間をミリ秒単位で返します。戻り値は`unsigned long`です。

```cpp
if (millis() - startMs >= waitMs) {
  // 指定時間が経過した後の処理
}
```

この引き算形式は、`millis()`が長時間後に0へ戻るオーバーフローにも比較的強い書き方です。

### `delay(ms)`と`delayMicroseconds(us)`

- `delay(20)`: 20ミリ秒待つ。
- `delayMicroseconds(250)`: 250マイクロ秒待つ。

待っている間は基本的に次の処理へ進みません。長い待ち時間が多いと、障害物への反応が遅くなります。

### `Serial`

USBシリアル通信を通して、パソコンのシリアルモニターへ情報を表示します。

- `Serial.begin(115200)`: 通信速度を115200 bpsにする。
- `Serial.print(value)`: 改行せず表示する。
- `Serial.println(value)`: 表示後に改行する。
- `F("text")`: 固定文字列をRAMではなくフラッシュメモリ側に置き、RAM消費を抑える。

## 4. C++の型と記法

### 基本型

| 型 | このコードでの用途 |
|---|---|
| `int` | ピン番号、センサー値、角度、誤差 |
| `long` | 複数のアナログ値を加算する変数 |
| `unsigned long` | `millis()`で扱う経過時間 |
| `float` | 距離、傾き、速度などの小数 |
| `bool` | 真偽値。`true`または`false` |
| `byte` | 0から255の1バイト値、I2Cアドレス |
| `int16_t` | 符号付き16ビット整数。MPUの生データ |
| `const char*` | 文字列の先頭を指すポインター |

### `const`

`const`を付けた変数は、初期化後に変更できません。

```cpp
const int PIN_SERVO = 10;
```

ピン番号や閾値を定数にすると、値の意味が分かりやすくなり、誤って変更することも防げます。

### 配列

同じ型の値を複数まとめて持つ仕組みです。

```cpp
int color[4];
```

要素番号は0から始まるため、4要素の番号は`0, 1, 2, 3`です。

### `enum`

関連する選択肢へ名前を付ける仕組みです。

```cpp
enum LineColor {
  LINE_BLACK,
  LINE_WHITE,
  LINE_OTHER
};
```

単なる数値よりも、`LINE_BLACK`のような名前の方が意味を読み取りやすくなります。

### `struct`

関連する複数の値を、1つのまとまりとして扱う仕組みです。

- `SensorData`: 1回分のセンサー情報
- `MotorCommand`: 1回分のモーターとサーボへの指令

### クラスとオブジェクト

クラスは、データと操作をひとまとめにした設計図です。オブジェクトは、その設計図から作った実体です。

```cpp
Servo steeringServo;
```

- `Servo`: クラス名
- `steeringServo`: 作成したオブジェクト名
- `steeringServo.write(...)`: オブジェクトが持つ関数を呼び出す

`Wire`と`Serial`も、Arduino側で用意されたオブジェクトとして使っています。

### 参照渡し `const SensorData&`

```cpp
MotorCommand updateController(const SensorData& sensor)
```

`&`を使うと、構造体全体をコピーせず元のデータを参照できます。`const`があるので、関数内から元データを変更できません。

### キャスト

```cpp
(int)(sum / SENSOR_SAMPLES)
```

値を明示的に別の型へ変換します。この例では計算結果を`int`として返します。

### 三項演算子

```cpp
bool forward = value >= 0.0 ? true : false;
```

`条件 ? 条件が真の値 : 条件が偽の値`という短い条件分岐です。

### ビット演算

`selectMux()`では、チャンネル番号の各ビットを取り出します。

```cpp
channel & 0x01
(channel >> 1) & 0x01
```

- `&`: 対応するビット同士のAND
- `>>`: ビットを右へずらす
- `0x01`: 16進数表記の1

3本の選択線で、2進数`000`から`111`の8チャンネルを指定できます。

## 5. このプログラムで定義した型

### `RobotState`

ロボットが現在どの行動をしているかを表します。このように状態に応じて処理を切り替える設計を「状態機械」または「ステートマシン」と呼びます。

| 状態 | 意味 |
|---|---|
| `STATE_INIT` | 起動直後 |
| `STATE_CALIBRATION` | センサーの安定待ち |
| `STATE_LINE_TRACE` | 通常のライン追従 |
| `STATE_CURVE_ENTRY` | カーブへ入った直後 |
| `STATE_CURVE_TRACE` | カーブ追従 |
| `STATE_OBSTACLE_DETECTED` | 障害物を検知した直後 |
| `STATE_OBSTACLE_AVOIDANCE` | 障害物回避中 |
| `STATE_LINE_RECOVERY` | 見失ったラインを探索中 |
| `STATE_TILT_RECOVERY` | 傾きの回復動作中 |
| `STATE_GOAL` | ゴールして停止 |
| `STATE_EMERGENCY_STOP` | 危険な傾きで緊急停止 |

### `LineColor`

センサー値を、黒、白、その中間の3種類に分類します。

### `SensorData`

```cpp
struct SensorData {
  int color[4];
  float distanceCm;
  float rollDeg;
};
```

4個のラインセンサー、障害物までの距離、ロール角をまとめています。

### `MotorCommand`

4輪の指令値、サーボ角度、停止フラグをまとめています。車輪の値は基本的に-1.0から1.0です。

## 6. 各関数の役割

| 関数 | 役割 |
|---|---|
| `setup()` | 通信、ピン、サーボ、MPUを初期化する |
| `loop()` | センサー入力、判断、出力を繰り返す |
| `updateController()` | 状態とセンサー値から次の指令を決める |
| `readSensors()` | 全センサー値を`SensorData`へまとめる |
| `readMuxAverage()` | マルチプレクサ経由で複数回読み、平均を返す |
| `selectMux()` | 74HC4051の読み取りチャンネルを選ぶ |
| `readDistanceCm()` | 距離センサー値をcmへ概算変換する |
| `initMpu()` | MPU-6050のスリープを解除する |
| `readRollDeg()` | MPUの加速度からロール角を計算する |
| `classifyLine()` | センサー値を黒、白、その他へ分類する |
| `isBlack()` | 指定センサーが黒か判定する |
| `allWhite()` | 4センサーすべてが白か判定する |
| `isGoalPattern()` | 4センサーすべてが黒か判定する |
| `calcLineError()` | 黒線の左右方向のずれを計算する |
| `makeStopCommand()` | 完全停止の指令を作る |
| `makeMecanumCommand()` | 前後、左右、旋回を4輪速度へ変換する |
| `applyMotorCommand()` | 指令を実際のサーボとモーターへ出力する |
| `ramp()` | 1周期あたりの速度変化を制限する |
| `writeMotor()` | 1個のモーターへ方向とPWMを出力する |
| `stopMotors()` | 全モーターを停止し、現在速度も0へ戻す |
| `setState()` | 状態を変更し、状態開始時刻を記録する |
| `constrainFloat()` | 小数値を最小値と最大値の範囲へ収める |
| `stateName()` | 状態をデバッグ表示用文字列へ変換する |
| `printDebug()` | センサー値と指令を一定間隔で表示する |

## 7. センサー処理

### 移動平均

`readMuxAverage()`は同じチャンネルを5回読み、合計を回数で割ります。瞬間的なノイズの影響を小さくできます。

### ライン判定

```cpp
value <= BLACK_THRESHOLD  // 黒
value >= WHITE_THRESHOLD  // 白
```

その間は`LINE_OTHER`です。照明、床、センサー高さによって値が変わるため、実機で閾値を調整する必要があります。

### ロール角

MPU-6050のY軸とZ軸の加速度から、次の考え方で傾きを求めます。

```cpp
roll = atan2(ayG, azG) * 57.2958;
```

`atan2()`の結果はラジアンなので、約57.2958を掛けて度へ変換します。加速度だけの計算は、走行中の振動や加減速の影響を受けます。

## 8. メカナムホイールの計算

入力は次の3つです。

- `vx`: 前後移動。正が前進
- `vy`: 左右移動。このコードでは符号に応じて横方向が変わる
- `omega`: 旋回。符号に応じて左右へ回る

4輪の速度は次のように合成します。

```cpp
fl = vx + vy + omega;
fr = vx - vy - omega;
rl = vx - vy + omega;
rr = vx + vy - omega;
```

計算結果の絶対値が1.0を超えた場合は、全車輪を同じ比率で割って-1.0から1.0へ正規化します。

## 9. 安全に学習・調整する順番

1. モーターを床から浮かせる。
2. シリアルモニターを115200 bpsで開く。
3. S1からS4の白と黒の値を記録する。
4. `BLACK_THRESHOLD`と`WHITE_THRESHOLD`を調整する。
5. 各モーターが正しい方向へ回るか確認する。
6. 逆なら対応する`*_DIRECTION_SIGN`を`-1`にする。
7. `BASE_SPEED`を低い値から少しずつ上げる。
8. ライン追従を確認してから障害物回避と傾き検知を試す。

サーボやモーターをArduinoの5 Vピンから直接駆動しないでください。外部電源を使う場合も、Arduino、ドライバー、サーボのGNDは共通にします。

## 10. 読み進める練習

最初は次の順番でコードを追うと理解しやすくなります。

1. `setup()`
2. `loop()`
3. `readSensors()`
4. `updateController()`
5. `makeMecanumCommand()`
6. `applyMotorCommand()`
7. `writeMotor()`

関数を読むときは、毎回「入力は何か」「戻り値は何か」「グローバル変数を変更するか」の3点を確認してください。
