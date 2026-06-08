# 部品仕様まとめ

## 確認した資料

- `/Users/adachiitsuki/Downloads/robot_architecture.pptx.pdf`
  - 指定名は`robot_arcitecture.pptx.pdf`だったが、実ファイルは`robot_architecture.pptx.pdf`。
- `/Users/adachiitsuki/Downloads/M_flowchart.png`
- `/Users/adachiitsuki/Downloads/mechatro1_week4_mon (1).pdf` 5ページ目
- 秋月電子の商品ページ
- Arduino UNO R3 datasheet
- MPU-6050 module資料

## ロボット概要資料から読み取った構成

- コンセプトは「倒れを検知して補正するバランス寄りロボット」。
- 元資料では前輪をサーボで操舵、後輪をDCモーターで駆動するバイク型構造。
- 両側にタミヤのボールキャスターを置き、転倒を防ぐ。
- 前方に距離センサーを置き、障害物を検知して回避する。
- カラーセンサーで黒/白を判定し、ラインを追従する。
- MPU-6050で傾きを読み、傾きが大きい場合は減速・停止する。

## 今回コードで採用した構成

課題文ではメカナムホイールが指定されているため、最終コードは4輪メカナムを主構成にした。サーボは障害物回避用の向き調整、または元資料の前輪操舵機構が残る場合の操舵補助として使う。

元資料と課題部品に差があるため、以下は仮定として実装した。

- メカナムホイールは4個、DCモーターも4個使う。
- DRV8835モータードライバーモジュールは2枚使う。
- UNOのアナログ入力不足を補うため、74HC4051相当のアナログマルチプレクサを追加する。
- カラーセンサーはRGB全chではなく、ライン検知に使いやすい1chまたは明度相当の電圧を各センサーから読む。

## 部品別まとめ

| 部品 | 仕様・接続 | 制御方法 | 注意点 |
|---|---|---|---|
| Arduino UNO R3/互換機 | ATmega328P、14デジタルI/O、6アナログ入力、I2CはA4/A5、VIN最大20V | Arduino IDEで書き込み。PWMは`analogWrite()` | モーターやサーボへ5Vピンから直接大電流を供給しない |
| MPU-6050 | 3軸加速度+3軸ジャイロ。多くのモジュールは3.3-5V対応、I2C接続 | `Wire`で0x68へアクセス。今回は加速度からroll角を概算 | 基板により3.3V専用の場合あり。VCC仕様を実物で確認 |
| RGBカラーセンサー S9032-02 | RGBフォトダイオード、アナログ素子 | 抵抗/アンプで電圧化して`analogRead()` | 素子単体なので直結だけでは安定しにくい。照明条件で閾値が変わる |
| 距離モジュール GP2Y0A21YK | Sharp赤外線測距、アナログ出力、10-80cm程度の用途 | 電圧を`analogRead()`し、概算式でcmへ変換 | 近距離/遠距離で非線形。実機で距離表を作る |
| MG996Rサーボ | 4.8-6.6V、PWM、180度 | Arduino `Servo`ライブラリ | 起動電流が大きいので別電源推奨。GNDはArduinoと共通化 |
| DCギアードモーター 6V 1:298 | 定格6V、無負荷35rpm、停動電流約140mA | DRV8835でPWM制御 | 4輪合計の停動電流を見込む。逆転方向は定数で補正 |
| DRV8835モータードライバー | 2ch DCモーター駆動。モーター電源は低電圧小型モーター向け | PWM+DIR相当で各モーターを制御 | 4輪メカナムには2枚必要。熱と電流制限に注意 |
| メカナムホイール | 4輪の速度合成で前後、左右、旋回が可能 | `vx, vy, omega`から4輪速度へ変換 | ローラー向きとモーター方向が合わないと横移動できない |
| タミヤ ボールキャスター | 補助支持・転倒抑制 | 制御不要 | 接地しすぎると駆動輪の荷重が抜ける |
| バッテリー | リンク先はAmazon短縮URLのため詳細未確認 | Arduino/モーター/サーボへ適切に分配 | 電圧、放電能力、保護回路、極性を実物で確認 |

## 必要ライブラリ

- `Wire`: Arduino標準。MPU-6050のI2C通信に使用。
- `Servo`: Arduino標準。MG996RのPWM制御に使用。

外部MPUライブラリは使わず、コンパイルしやすさを優先してレジスタ直読みとした。

## 参照URL

- Arduino UNO R3 datasheet: https://docs.arduino.cc/resources/datasheets/A000066-datasheet.pdf
- RGBカラーセンサー S9032-02: https://akizukidenshi.com/catalog/g/g109763/
- GP2Y0A21YK: https://akizukidenshi.com/catalog/g/g102551/
- MG996R: https://akizukidenshi.com/catalog/g/g112534/
- DCギアードモーター: https://akizukidenshi.com/catalog/g/g115144/
- DRV8835モータードライバー: https://akizukidenshi.com/catalog/g/g109848/
- MPU-6050 module参考: https://www.sciallastore.com/wp-content/uploads/2020/10/MPU-6050-Module.pdf
- メカナム運動学参考: https://frcdocs.wpi.edu/en/latest/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html
