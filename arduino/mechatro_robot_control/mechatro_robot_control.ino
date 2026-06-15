#include <Servo.h>  // サーボモーターを角度指定で制御するServoクラスを読み込む。
#include <Wire.h>   // MPU-6050とI2C通信するためのWire機能を読み込む。

/*
  Mechatro robot control

  重要な仮定:
  - 指定資料名 robot_arcitecture.pptx.pdf は見つからず、Downloads内の
    robot_architecture.pptx.pdf を確認した。
  - フローチャートはS1..S4の4個ラインセンサーを前提としている。
  - Arduino UNOでは4個カラーセンサー + 距離センサー + MPU(I2C)を直結する
    アナログ入力が不足するため、74HC4051等のアナログマルチプレクサを使う前提。
  - S9032-02はフォトダイオード素子なので、本来は抵抗またはOPアンプで
    電圧化した出力をanalogReadする。ここでは「黒ほど値が小さい」と仮定。
  - メカナム4輪はDRV8835を2枚使い、PH/EN相当のPWM+DIRで各DCモーターを制御する。
*/

// ===== ピン設定: 実機配線に合わせて最初に確認する =====
const int PIN_MUX_SIG = A0;     // 74HC4051の共通出力
const int PIN_MUX_S0 = 2;        // マルチプレクサの選択ビット0を出力するピン
const int PIN_MUX_S1 = 4;        // マルチプレクサの選択ビット1を出力するピン
const int PIN_MUX_S2 = A1;      // アナログ端子をデジタル出力として使用

const int PIN_SERVO = 10;        // サーボの信号線を接続するピン

// ServoライブラリはUNOのTimer1を使うため、D9/D10のPWMは避ける。
const int PIN_FL_PWM = 3;       // Front Left
const int PIN_FL_DIR = 7;        // 左前モーターの回転方向を決めるピン
const int PIN_FR_PWM = 5;       // Front Right
const int PIN_FR_DIR = 8;        // 右前モーターの回転方向を決めるピン
const int PIN_RL_PWM = 6;       // Rear Left
const int PIN_RL_DIR = 12;       // 左後モーターの回転方向を決めるピン
const int PIN_RR_PWM = 11;      // Rear Right
const int PIN_RR_DIR = 13;       // 右後モーターの回転方向を決めるピン

const byte MPU_ADDR = 0x68;      // MPU-6050の標準I2Cアドレス

// マルチプレクサのチャンネル割当
const int CH_COLOR_S1 = 0;       // ラインセンサーS1を接続するチャンネル
const int CH_COLOR_S2 = 1;       // ラインセンサーS2を接続するチャンネル
const int CH_COLOR_S3 = 2;       // ラインセンサーS3を接続するチャンネル
const int CH_COLOR_S4 = 3;       // ラインセンサーS4を接続するチャンネル
const int CH_DISTANCE = 4;       // 距離センサーを接続するチャンネル

// ===== 調整パラメータ: 実機テストで必ず変更する =====
const int BLACK_THRESHOLD = 430;       // これ未満を黒
const int WHITE_THRESHOLD = 720;       // これ以上を白
const int SENSOR_SAMPLES = 5;          // 平均値を求めるために読む回数

const float BASE_SPEED = 0.28;         // 初期は低速
const float CURVE_SPEED = 0.22;
const float RECOVERY_SPEED = 0.18;
const float MAX_SPEED_STEP = 0.035;    // 急加速防止

const float OBSTACLE_DISTANCE_CM = 18.0;
const float TILT_RECOVER_DEG = 15.0;
const float TILT_STOP_DEG = 30.0;

const unsigned long CALIBRATION_MS = 2500;    // 起動後にセンサーが安定するまで待つ時間
const unsigned long GOAL_CONFIRM_MS = 700;    // ゴール模様の連続検出を確認する時間
const unsigned long OBSTACLE_AVOID_MS = 900;  // 障害物回避動作を続ける時間
const unsigned long SERIAL_INTERVAL_MS = 200; // デバッグ表示の間隔

const int SERVO_CENTER = 90;     // サーボの中央角度
const int SERVO_LEFT = 60;       // サーボを左へ向ける角度
const int SERVO_RIGHT = 120;     // サーボを右へ向ける角度

// モーター配線が逆なら該当値を -1 にする
const int FL_DIRECTION_SIGN = 1; // 左前モーターの向きを補正する係数
const int FR_DIRECTION_SIGN = 1; // 右前モーターの向きを補正する係数
const int RL_DIRECTION_SIGN = 1; // 左後モーターの向きを補正する係数
const int RR_DIRECTION_SIGN = 1; // 右後モーターの向きを補正する係数

enum RobotState {
  // ロボットの「今の行動」を表す。loop()ではこの状態を見て次の動きを決める。
  STATE_INIT,                // 起動直後
  STATE_CALIBRATION,         // センサーの安定待ち
  STATE_LINE_TRACE,          // 通常のライン追従
  STATE_CURVE_ENTRY,         // カーブへ入った直後
  STATE_CURVE_TRACE,         // カーブ追従中
  STATE_OBSTACLE_DETECTED,   // 障害物を検知した直後
  STATE_OBSTACLE_AVOIDANCE,  // 障害物を回避中
  STATE_LINE_RECOVERY,       // 見失ったラインを探索中
  STATE_TILT_RECOVERY,       // 傾きを回復中
  STATE_GOAL,                // ゴールして停止中
  STATE_EMERGENCY_STOP       // 危険な傾きによる緊急停止
};

enum LineColor {
  LINE_BLACK,  // 黒線と判定した状態
  LINE_WHITE,  // 白い床と判定した状態
  LINE_OTHER   // 黒と白の閾値の中間
};

struct SensorData {
  // 1回の制御ループで使うセンサー値をまとめて持つ。
  int color[4];       // S1からS4までのアナログ値
  float distanceCm;   // 障害物までの推定距離[cm]
  float rollDeg;      // 左右方向の推定傾斜角[度]
};

struct MotorCommand {
  float fl;       // 左前輪の指令値。基本範囲は-1.0から1.0
  float fr;       // 右前輪の指令値
  float rl;       // 左後輪の指令値
  float rr;       // 右後輪の指令値
  int servoDeg;   // サーボへ指示する角度
  bool stop;      // trueなら他の値に関係なく全輪を停止する
};

Servo steeringServo;                    // 操舵用サーボを操作するオブジェクト
RobotState state = STATE_INIT;           // 現在のロボット状態
unsigned long stateEnteredMs = 0;        // 現在の状態へ入った時刻
unsigned long goalCandidateMs = 0;       // ゴール模様を最初に検出した時刻
unsigned long lastSerialMs = 0;          // 最後にデバッグ表示した時刻
int lastLineError = 0;                   // ラインを失ったときにも使う直前の左右誤差
float currentFl = 0.0;                   // 急加速制限後の左前輪指令
float currentFr = 0.0;                   // 急加速制限後の右前輪指令
float currentRl = 0.0;                   // 急加速制限後の左後輪指令
float currentRr = 0.0;                   // 急加速制限後の右後輪指令

/*
  関数: setup
  役割: 通信、出力ピン、サーボ、MPU-6050を初期化する。
  実行: 電源投入時またはリセット時に1回だけ呼ばれる。
*/

void setup() {
  // setup()は電源投入後に1回だけ動く。通信、ピン、センサーの初期設定を行う。
  Serial.begin(115200);  // パソコンとのシリアル通信を115200bpsで開始する。
  Wire.begin();          // I2C通信を開始する。

  pinMode(PIN_MUX_S0, OUTPUT);  // マルチプレクサ選択線S0を出力にする。
  pinMode(PIN_MUX_S1, OUTPUT);  // マルチプレクサ選択線S1を出力にする。
  pinMode(PIN_MUX_S2, OUTPUT);  // マルチプレクサ選択線S2を出力にする。

  pinMode(PIN_FL_PWM, OUTPUT);  // 左前輪の速度制御ピンを出力にする。
  pinMode(PIN_FL_DIR, OUTPUT);  // 左前輪の方向制御ピンを出力にする。
  pinMode(PIN_FR_PWM, OUTPUT);  // 右前輪の速度制御ピンを出力にする。
  pinMode(PIN_FR_DIR, OUTPUT);  // 右前輪の方向制御ピンを出力にする。
  pinMode(PIN_RL_PWM, OUTPUT);  // 左後輪の速度制御ピンを出力にする。
  pinMode(PIN_RL_DIR, OUTPUT);  // 左後輪の方向制御ピンを出力にする。
  pinMode(PIN_RR_PWM, OUTPUT);  // 右後輪の速度制御ピンを出力にする。
  pinMode(PIN_RR_DIR, OUTPUT);  // 右後輪の方向制御ピンを出力にする。

  steeringServo.attach(PIN_SERVO);     // Servoオブジェクトと信号ピンを関連付ける。
  steeringServo.write(SERVO_CENTER);   // 起動時のサーボを中央へ向ける。
  stopMotors();                        // 初期化中に車輪が回らないよう停止する。
  initMpu();                           // MPU-6050をスリープ解除する。

  setState(STATE_CALIBRATION);  // 最初の動作状態をセンサー安定待ちにする。
  Serial.println(F("mechatro_robot_control start"));  // 起動完了を表示する。
}

/*
  関数: loop
  役割: センサー入力、危険判定、制御計算、実機出力、デバッグ表示を繰り返す。
  実行周期: 最後のdelay(20)により、おおよそ50Hzを目標にする。
*/
void loop() {
  // loop()は繰り返し動く。読む -> 判断する -> モーターへ出す、の順で制御する。
  SensorData sensor = readSensors();  // 今回の制御周期で使う全センサー値を取得する。

  if (abs(sensor.rollDeg) >= TILT_STOP_DEG) {
    setState(STATE_EMERGENCY_STOP);  // 危険角度なら停止状態へ移る。
  }

  MotorCommand command = updateController(sensor);  // センサー値から次の動作指令を計算する。
  applyMotorCommand(command);                       // 計算した指令をモーターとサーボへ出す。
  printDebug(sensor, command);                      // 状態確認用の情報を必要な周期で表示する。

  delay(20);  // 約50Hz制御。センサーが不安定なら30-40msへ増やす。
}

/*
  関数: updateController
  引数: sensor - 今回読み取ったセンサー値。const参照なのでコピーも書き換えもしない。
  戻り値: 4輪速度、サーボ角度、停止指定をまとめたMotorCommand。
  役割: 状態機械の中心として、センサー値から次の動きを決定する。
*/
MotorCommand updateController(const SensorData& sensor) {
  // センサー値と現在の状態から、次に出すモーター指令を決める中心部分。
  if (state == STATE_EMERGENCY_STOP || state == STATE_GOAL) {
    return makeStopCommand();  // 停止を維持する指令を返し、以降の判定を行わない。
  }

  if (state == STATE_CALIBRATION) {
    // 起動直後は少し待つ。センサー値が落ち着くまで走らせないため。
    if (millis() - stateEnteredMs >= CALIBRATION_MS) {  // 安定待ち時間を過ぎたか調べる。
      setState(STATE_LINE_TRACE);  // 通常のライン追従へ移る。
    }
    return makeStopCommand();  // 待機中はモーターを停止する。
  }

  if (abs(sensor.rollDeg) >= TILT_RECOVER_DEG) {
    // 大きく傾いたら、ライン追従より姿勢の立て直しを優先する。
    setState(STATE_TILT_RECOVERY);  // 傾き回復を最優先の状態にする。
  } else if (sensor.distanceCm > 0.0 && sensor.distanceCm <= OBSTACLE_DISTANCE_CM) {
    setState(STATE_OBSTACLE_DETECTED);  // 有効距離内の障害物を検出した状態にする。
  }

  if (isGoalPattern(sensor)) {
    // 一瞬の誤検出で止まらないよう、ゴール模様が一定時間続いたときだけゴール扱いにする。
    if (goalCandidateMs == 0) {
      goalCandidateMs = millis();  // ゴール候補を最初に見つけた時刻を記録する。
    }
    if (millis() - goalCandidateMs >= GOAL_CONFIRM_MS) {
      setState(STATE_GOAL);       // 一定時間続いたのでゴール状態にする。
      return makeStopCommand();   // その周期から直ちに停止する。
    }
  } else {
    goalCandidateMs = 0;  // 模様が途切れたのでゴール確認時間をリセットする。
  }

  if (state == STATE_TILT_RECOVERY) {
    if (abs(sensor.rollDeg) < TILT_RECOVER_DEG * 0.6) {
      setState(STATE_LINE_TRACE);  // 傾きが十分小さくなったら通常走行へ戻る。
    }
    float omega = sensor.rollDeg > 0.0 ? -0.18 : 0.18;  // 傾きと反対方向の旋回量を選ぶ。
    return makeMecanumCommand(RECOVERY_SPEED * 0.5, 0.0, omega, SERVO_CENTER);  // 低速で姿勢を戻す。
  }

  if (state == STATE_OBSTACLE_DETECTED) {
    setState(STATE_OBSTACLE_AVOIDANCE);  // 次の周期から回避動作を始める。
    return makeStopCommand();            // 検出直後の1周期は一度停止する。
  }

  if (state == STATE_OBSTACLE_AVOIDANCE) {
    if (millis() - stateEnteredMs >= OBSTACLE_AVOID_MS) {
      setState(STATE_LINE_RECOVERY);  // 規定時間後はライン探索へ進む。
    }
    // 右へ横移動しつつ低速前進。障害物が固定の場合は距離閾値と時間を短めに調整する。
    return makeMecanumCommand(RECOVERY_SPEED, -0.18, 0.10, SERVO_RIGHT);
  }

  if (state == STATE_LINE_RECOVERY) {
    if (!allWhite(sensor)) {
      setState(STATE_LINE_TRACE);  // 白以外を検出したらライン追従へ戻る。
    }
    float omega = lastLineError >= 0 ? -0.22 : 0.22;  // 最後に線があった向きへ旋回する。
    int servo = lastLineError >= 0 ? SERVO_LEFT : SERVO_RIGHT;  // 探索方向へサーボを向ける。
    return makeMecanumCommand(RECOVERY_SPEED, 0.0, omega, servo);  // 低速前進しながら探索する。
  }

  if (allWhite(sensor)) {
    setState(STATE_LINE_RECOVERY);  // 全白ならラインを見失ったと判断する。
    return makeMecanumCommand(0.0, 0.0, lastLineError >= 0 ? -0.18 : 0.18, SERVO_CENTER);
  }

  bool s1 = isBlack(sensor, 0);  // 左端S1が黒かを保存する。
  bool s2 = isBlack(sensor, 1);  // 左中央S2が黒かを保存する。
  bool s3 = isBlack(sensor, 2);  // 右中央S3が黒かを保存する。
  bool s4 = isBlack(sensor, 3);  // 右端S4が黒かを保存する。

  if ((s1 || s4) && !(s2 && s3)) {
    setState(STATE_CURVE_ENTRY);  // 外側センサー反応をカーブ進入として扱う。
  }
  if ((s1 && s4) && !s2 && !s3) {
    // フローチャートの「S1&S4黒、S2&S3白」はカーブ完了/通常復帰と解釈。
    setState(STATE_LINE_TRACE);  // カーブ完了パターンなら通常追従へ戻す。
  } else if (state == STATE_CURVE_ENTRY) {
    setState(STATE_CURVE_TRACE);  // 進入の次はカーブ追従状態へ移す。
  }

  int error = calcLineError(sensor);  // 黒線が中央からどれだけずれたか計算する。
  lastLineError = error;              // ラインを失った場合に備えて誤差を保存する。
  float gain = (state == STATE_CURVE_TRACE) ? 0.085 : 0.055;  // カーブ中は補正を強くする。
  float speed = (state == STATE_CURVE_TRACE) ? CURVE_SPEED : BASE_SPEED;  // カーブ中は減速する。
  float omega = constrainFloat(-gain * error, -0.45, 0.45);  // 誤差を旋回量へ変換して上限を設ける。
  int servo = constrain(SERVO_CENTER - error * 7, SERVO_LEFT, SERVO_RIGHT);  // 誤差を操舵角へ変換する。

  return makeMecanumCommand(speed, 0.0, omega, servo);  // 前進、旋回、操舵を1つの指令にする。
}

/*
  関数: readSensors
  戻り値: 4個のライン値、距離、ロール角をまとめたSensorData。
  役割: 1回の制御で使う入力値を同じ時点のデータとしてまとめる。
*/
SensorData readSensors() {
  // ここで全センサーを読み、他の関数が扱いやすい形にまとめる。
  SensorData data;  // 戻り値として使う構造体を用意する。
  data.color[0] = readMuxAverage(CH_COLOR_S1);  // S1を複数回読んだ平均値を入れる。
  data.color[1] = readMuxAverage(CH_COLOR_S2);  // S2を複数回読んだ平均値を入れる。
  data.color[2] = readMuxAverage(CH_COLOR_S3);  // S3を複数回読んだ平均値を入れる。
  data.color[3] = readMuxAverage(CH_COLOR_S4);  // S4を複数回読んだ平均値を入れる。
  data.distanceCm = readDistanceCm();           // 距離をcmへ変換して入れる。
  data.rollDeg = readRollDeg();                 // MPUから求めた傾斜角を入れる。
  return data;                                  // 完成したセンサーデータを呼び出し元へ返す。
}

/*
  関数: readMuxAverage
  引数: channel - 74HC4051で選択するチャンネル番号。
  戻り値: SENSOR_SAMPLES回読んだアナログ値の平均。
*/
int readMuxAverage(int channel) {
  // 74HC4051で読む入力を切り替え、数回読んだ平均を使ってノイズを減らす。
  selectMux(channel);       // 読みたい入力へマルチプレクサを切り替える。
  delayMicroseconds(250);   // 切り替え後の電圧が安定するまで短く待つ。
  long sum = 0;             // 複数回の値を足す変数を0で初期化する。
  for (int i = 0; i < SENSOR_SAMPLES; i++) {  // 指定回数だけ繰り返す。
    sum += analogRead(PIN_MUX_SIG);  // アナログ値を読み、合計へ加える。
    delayMicroseconds(200);          // 次の読み取りまで短く待つ。
  }
  return (int)(sum / SENSOR_SAMPLES);  // 合計を回数で割った整数平均を返す。
}

/*
  関数: selectMux
  引数: channel - 0から7のチャンネル番号。
  役割: チャンネル番号の下位3ビットをS0、S1、S2へ出力する。
*/
void selectMux(int channel) {
  digitalWrite(PIN_MUX_S0, channel & 0x01);         // 1ビット目をS0へ出す。
  digitalWrite(PIN_MUX_S1, (channel >> 1) & 0x01);  // 2ビット目をS1へ出す。
  digitalWrite(PIN_MUX_S2, (channel >> 2) & 0x01);  // 3ビット目をS2へ出す。
}

/*
  関数: readDistanceCm
  戻り値: 距離センサーから概算した距離[cm]。
  注意: 変換式は近似なので、実機測定による補正が必要。
*/
float readDistanceCm() {
  int raw = readMuxAverage(CH_DISTANCE);  // 距離センサーのADC値を読む。
  float voltage = raw * (5.0 / 1023.0);   // 0から1023の値を0から5Vへ換算する。
  // GP2Y0A21YKの概算式。10-80cm付近以外では誤差が大きいので実機で表を作る。
  if (voltage <= 0.42) {
    return 80.0;  // 電圧が低すぎる場合は測定上限付近として扱う。
  }
  float cm = 27.86 * pow(voltage, -1.15);  // センサー特性の近似式でcmを求める。
  return constrainFloat(cm, 6.0, 80.0);    // 異常な結果を想定範囲へ収めて返す。
}

/*
  関数: initMpu
  役割: MPU-6050の電源管理レジスタへ0を書き、スリープ状態を解除する。
*/
void initMpu() {
  Wire.beginTransmission(MPU_ADDR);  // MPU-6050へのI2C送信を開始する。
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // wake up
  Wire.endTransmission(true);  // バッファの2バイトを送信し、通信を終了する。
}

/*
  関数: readRollDeg
  戻り値: Y軸とZ軸の加速度から求めたロール角[度]。
  役割: MPU-6050の加速度6バイトを読み、左右方向の傾きを概算する。
*/
float readRollDeg() {
  Wire.beginTransmission(MPU_ADDR);  // MPU-6050への送信を開始する。
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    return 0.0;      // MPU未接続時は停止させたい場合、ここで大角度を返す。
  }
  Wire.requestFrom(MPU_ADDR, (byte)6, (byte)true);  // X、Y、Z軸の加速度6バイトを要求する。
  if (Wire.available() < 6) {
    return 0.0;  // 必要なデータが届かなければ安全な既定値を返す。
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();  // X軸の上位・下位バイトを16ビット値へ結合する。
  int16_t ay = (Wire.read() << 8) | Wire.read();  // Y軸の上位・下位バイトを結合する。
  int16_t az = (Wire.read() << 8) | Wire.read();  // Z軸の上位・下位バイトを結合する。

  float ayG = ay / 16384.0;                    // 初期感度設定を前提にY軸値を重力加速度単位へ直す。
  float azG = az / 16384.0;                    // Z軸値も重力加速度単位へ直す。
  float roll = atan2(ayG, azG) * 57.2958;      // atan2のラジアン結果を度へ変換する。
  (void)ax;                                    // 今回使わないaxに対する警告を抑える。
  return roll;                                 // 計算したロール角を返す。
}

/*
  関数: classifyLine
  引数: value - ラインセンサーのアナログ値。
  戻り値: LINE_BLACK、LINE_WHITE、LINE_OTHERのいずれか。
*/
LineColor classifyLine(int value) {
  if (value <= BLACK_THRESHOLD) {
    return LINE_BLACK;  // 黒閾値以下なら黒と判定する。
  }
  if (value >= WHITE_THRESHOLD) {
    return LINE_WHITE;  // 白閾値以上なら白と判定する。
  }
  return LINE_OTHER;  // どちらの閾値にも入らなければ中間とする。
}

/*
  関数: isBlack
  引数: sensor - センサーデータ、index - S1からS4に対応する0から3。
  戻り値: 指定したセンサーが黒ならtrue。
*/
bool isBlack(const SensorData& sensor, int index) {
  return classifyLine(sensor.color[index]) == LINE_BLACK;  // 分類結果と黒を比較する。
}

/*
  関数: allWhite
  戻り値: 4個のラインセンサーがすべて白ならtrue。
*/
bool allWhite(const SensorData& sensor) {
  for (int i = 0; i < 4; i++) {  // S1からS4まで順番に調べる。
    if (classifyLine(sensor.color[i]) != LINE_WHITE) {
      return false;  // 1個でも白以外なら、その時点でfalseを返す。
    }
  }
  return true;  // 全要素の確認を通過したのでtrueを返す。
}

/*
  関数: isGoalPattern
  戻り値: 4個のラインセンサーが同時に黒ならtrue。
*/
bool isGoalPattern(const SensorData& sensor) {
  // コース図のゴールはスタート付近の横線。4センサー同時黒が一定時間続いたら停止。
  return isBlack(sensor, 0) && isBlack(sensor, 1) && isBlack(sensor, 2) && isBlack(sensor, 3);
}

/*
  関数: calcLineError
  戻り値: 黒線位置の平均。負は左、正は右、0は中央を表す。
  役割: 各センサーへ重みを付け、操舵と旋回に使う誤差を求める。
*/
int calcLineError(const SensorData& sensor) {
  // 黒線が中央からどれだけ左右にずれているかを、負=左、正=右で表す。
  const int weights[4] = {-3, -1, 1, 3};  // S1,S2,S3,S4を左から右へ並べた仮定。
  int sum = 0;    // 反応したセンサーの重み合計
  int count = 0;  // 黒と判定したセンサー数
  for (int i = 0; i < 4; i++) {  // 4センサーを左から右へ確認する。
    if (isBlack(sensor, i)) {
      sum += weights[i];  // 黒だった位置の重みを加える。
      count++;            // 黒だったセンサー数を1増やす。
    }
  }
  if (count == 0) {
    return lastLineError;  // 黒がなければ直前の方向を維持する。
  }
  return sum / count;  // 重みの平均を整数のライン誤差として返す。
}

/*
  関数: makeStopCommand
  戻り値: 全輪0、サーボ中央、停止フラグtrueの指令。
*/
MotorCommand makeStopCommand() {
  MotorCommand cmd;              // 作成する指令構造体を用意する。
  cmd.fl = 0.0;                  // 左前輪を停止する。
  cmd.fr = 0.0;                  // 右前輪を停止する。
  cmd.rl = 0.0;                  // 左後輪を停止する。
  cmd.rr = 0.0;                  // 右後輪を停止する。
  cmd.servoDeg = SERVO_CENTER;   // サーボを中央へ戻す。
  cmd.stop = true;               // 強制停止指令であることを示す。
  return cmd;                    // 完成した停止指令を返す。
}

/*
  関数: makeMecanumCommand
  引数: vx - 前後、vy - 左右、omega - 旋回、servoDeg - サーボ角度。
  戻り値: 4輪へ分配し、最大絶対値を1.0以下へ正規化した指令。
*/
MotorCommand makeMecanumCommand(float vx, float vy, float omega, int servoDeg) {
  // vx=前後、vy=左右、omega=旋回。4輪それぞれの強さに変換する。
  MotorCommand cmd;                  // 計算結果を入れる指令を用意する。
  cmd.fl = vx + vy + omega;          // 左前輪の速度を合成する。
  cmd.fr = vx - vy - omega;          // 右前輪の速度を合成する。
  cmd.rl = vx - vy + omega;          // 左後輪の速度を合成する。
  cmd.rr = vx + vy - omega;          // 右後輪の速度を合成する。

  float maxAbs = max(1.0, max(max(abs(cmd.fl), abs(cmd.fr)), max(abs(cmd.rl), abs(cmd.rr))));  // 最大絶対値を求める。
  cmd.fl /= maxAbs;             // 比率を保ったまま左前輪を範囲内へ収める。
  cmd.fr /= maxAbs;             // 右前輪も同じ値で割る。
  cmd.rl /= maxAbs;             // 左後輪も同じ値で割る。
  cmd.rr /= maxAbs;             // 右後輪も同じ値で割る。
  cmd.servoDeg = servoDeg;      // 呼び出し側が指定した操舵角を保存する。
  cmd.stop = false;             // 通常の走行指令であることを示す。
  return cmd;                   // 完成した指令を返す。
}

/*
  関数: applyMotorCommand
  引数: cmd - 制御計算で作られたモーターとサーボの指令。
  役割: 指令を急加速制限へ通し、実際の出力ピンへ反映する。
*/
void applyMotorCommand(const MotorCommand& cmd) {
  // 計算した指令を実際のサーボ角度とモーターPWMへ反映する。
  steeringServo.write(constrain(cmd.servoDeg, SERVO_LEFT, SERVO_RIGHT));  // 安全範囲内の角度だけを出力する。

  if (cmd.stop) {
    stopMotors();  // 停止フラグがあれば全輪を直ちに止める。
    return;        // 以降の走行出力を行わず関数を終える。
  }

  currentFl = ramp(currentFl, cmd.fl);  // 左前輪を目標値へ少しずつ近づける。
  currentFr = ramp(currentFr, cmd.fr);  // 右前輪を目標値へ少しずつ近づける。
  currentRl = ramp(currentRl, cmd.rl);  // 左後輪を目標値へ少しずつ近づける。
  currentRr = ramp(currentRr, cmd.rr);  // 右後輪を目標値へ少しずつ近づける。

  writeMotor(PIN_FL_PWM, PIN_FL_DIR, currentFl * FL_DIRECTION_SIGN);  // 左前輪へ速度と方向を出す。
  writeMotor(PIN_FR_PWM, PIN_FR_DIR, currentFr * FR_DIRECTION_SIGN);  // 右前輪へ速度と方向を出す。
  writeMotor(PIN_RL_PWM, PIN_RL_DIR, currentRl * RL_DIRECTION_SIGN);  // 左後輪へ速度と方向を出す。
  writeMotor(PIN_RR_PWM, PIN_RR_DIR, currentRr * RR_DIRECTION_SIGN);  // 右後輪へ速度と方向を出す。
}

/*
  関数: ramp
  引数: current - 現在値、target - 目標値。
  戻り値: 1周期の変化量をMAX_SPEED_STEP以下にした次の値。
*/
float ramp(float current, float target) {
  if (target > current + MAX_SPEED_STEP) {
    return current + MAX_SPEED_STEP;  // 増加量を上限値に制限する。
  }
  if (target < current - MAX_SPEED_STEP) {
    return current - MAX_SPEED_STEP;  // 減少量も上限値に制限する。
  }
  return target;  // 目標が十分近ければ、そのまま目標値へ合わせる。
}

/*
  関数: writeMotor
  引数: pwmPin - 速度ピン、dirPin - 方向ピン、value - -1.0から1.0の指令。
  役割: 符号を方向へ、絶対値を0から255のPWMへ変換する。
*/
void writeMotor(int pwmPin, int dirPin, float value) {
  value = constrainFloat(value, -1.0, 1.0);  // 範囲外の指令を切り詰める。
  bool forward = value >= 0.0;               // 0以上なら正転と判定する。
  int pwm = (int)(abs(value) * 255.0);        // 指令の大きさを8ビットPWM値へ変換する。
  digitalWrite(dirPin, forward ? HIGH : LOW); // 三項演算子で回転方向を出力する。
  analogWrite(pwmPin, pwm);                   // PWMのON時間比でモーター速度を調整する。
}

/*
  関数: stopMotors
  役割: 全輪の現在値とPWM出力を0にし、確実に停止させる。
*/
void stopMotors() {
  currentFl = 0.0;                // 左前輪の記録値を0へ戻す。
  currentFr = 0.0;                // 右前輪の記録値を0へ戻す。
  currentRl = 0.0;                // 左後輪の記録値を0へ戻す。
  currentRr = 0.0;                // 右後輪の記録値を0へ戻す。
  analogWrite(PIN_FL_PWM, 0);     // 左前輪のPWMをOFFにする。
  analogWrite(PIN_FR_PWM, 0);     // 右前輪のPWMをOFFにする。
  analogWrite(PIN_RL_PWM, 0);     // 左後輪のPWMをOFFにする。
  analogWrite(PIN_RR_PWM, 0);     // 右後輪のPWMをOFFにする。
}

/*
  関数: setState
  引数: next - 次に設定したいRobotState。
  役割: 状態が変わる場合だけ更新し、その状態へ入った時刻も記録する。
*/
void setState(RobotState next) {
  if (state != next) {
    state = next;               // 現在状態を新しい状態へ置き換える。
    stateEnteredMs = millis();  // 状態ごとの経過時間を測る開始時刻を記録する。
  }
}

/*
  関数: constrainFloat
  戻り値: valueをminValue以上、maxValue以下へ収めた値。
  補足: Arduinoのconstrain()と同じ考え方をfloat用に明示した関数。
*/
float constrainFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;  // 下限未満なら下限値を返す。
  }
  if (value > maxValue) {
    return maxValue;  // 上限より大きければ上限値を返す。
  }
  return value;  // 範囲内なら元の値を返す。
}

/*
  関数: stateName
  引数: value - 文字列へ変換したいRobotState。
  戻り値: シリアル表示用の文字列。
*/
const char* stateName(RobotState value) {
  switch (value) {
    case STATE_INIT: return "INIT";
    case STATE_CALIBRATION: return "CALIBRATION";
    case STATE_LINE_TRACE: return "LINE_TRACE";
    case STATE_CURVE_ENTRY: return "CURVE_ENTRY";
    case STATE_CURVE_TRACE: return "CURVE_TRACE";
    case STATE_OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
    case STATE_OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOIDANCE";
    case STATE_LINE_RECOVERY: return "LINE_RECOVERY";
    case STATE_TILT_RECOVERY: return "TILT_RECOVERY";
    case STATE_GOAL: return "GOAL";
    case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";  // 定義外の値が渡された場合の表示名
}

/*
  関数: printDebug
  引数: sensor - 入力値、cmd - 今回計算した出力指令。
  役割: 状態、センサー、モーター、サーボを一定間隔で1行表示する。
*/
void printDebug(const SensorData& sensor, const MotorCommand& cmd) {
  if (millis() - lastSerialMs < SERIAL_INTERVAL_MS) {
    return;  // 前回表示から時間が短ければ通信量を抑えるため何もしない。
  }
  lastSerialMs = millis();  // 今回の表示時刻を保存する。

  Serial.print(F("state="));       // 状態ラベルを表示する。
  Serial.print(stateName(state));  // 現在状態を名前で表示する。
  Serial.print(F(" S="));          // センサー値の見出しを表示する。
  for (int i = 0; i < 4; i++) {    // S1からS4を順番に表示する。
    Serial.print(sensor.color[i]);  // センサーの数値を表示する。
    Serial.print(i == 3 ? ' ' : ',');  // 最後以外はカンマ、最後は空白で区切る。
  }
  Serial.print(F("dist="));           // 距離の見出しを表示する。
  Serial.print(sensor.distanceCm, 1); // 距離を小数点以下1桁で表示する。
  Serial.print(F(" roll="));          // ロール角の見出しを表示する。
  Serial.print(sensor.rollDeg, 1);    // ロール角を小数点以下1桁で表示する。
  Serial.print(F(" motor="));         // 4輪指令の見出しを表示する。
  Serial.print(cmd.fl, 2);            // 左前輪指令を小数点以下2桁で表示する。
  Serial.print(',');                  // 値の区切り文字を表示する。
  Serial.print(cmd.fr, 2);            // 右前輪指令を表示する。
  Serial.print(',');                  // 値の区切り文字を表示する。
  Serial.print(cmd.rl, 2);            // 左後輪指令を表示する。
  Serial.print(',');                  // 値の区切り文字を表示する。
  Serial.print(cmd.rr, 2);            // 右後輪指令を表示する。
  Serial.print(F(" servo="));         // サーボ角度の見出しを表示する。
  Serial.println(cmd.servoDeg);       // サーボ角度を表示し、最後に改行する。
}
