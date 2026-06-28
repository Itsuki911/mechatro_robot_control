# Arduino CLI / VS Code セットアップ

## インストール

macOSではHomebrewが使える場合、Arduino CLIをインストールします。

```sh
brew install arduino-cli
arduino-cli version
```

初回は設定ファイルとボード情報を更新します。

```sh
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr
```

## Arduino UNO向けコマンド

```sh
arduino-cli compile --fqbn arduino:avr:uno arduino/mechatro_robot_control
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:uno arduino/mechatro_robot_control
arduino-cli monitor -p /dev/cu.usbmodemXXXX -c baudrate=115200
```

ポート確認:

```sh
arduino-cli board list
```

## 補助スクリプト

```sh
scripts/arduino_compile.sh
PORT=/dev/cu.usbmodemXXXX scripts/arduino_upload.sh
PORT=/dev/cu.usbmodemXXXX scripts/arduino_monitor.sh
```

`FQBN` を環境変数で変えると互換ボードにも流用できます。

## VS Code Tasks

`.vscode/tasks.json` に以下を用意しています。

- Arduino CLI: compile
- Arduino CLI: upload
- Python: serial logger
- Python: analyze dummy clean

推奨拡張は `.vscode/extensions.json` にあります。
