# procon26 mvp

このリポジトリは、M5Stack を使った BLE 連携の試作プロジェクトです。`ble_send` と `m5` はどちらもプロトタイプであり、実装や仕様は今後変更される前提です。

## 構成

### `ble_send`

M5Stack CoreS3 で動作する送信側プロトタイプです。`M5_BEACON` という名前で BLE Beacon を広告し、受信側が RSSI を使って距離の目安を取得できるようにしています。

- 対象ボード: M5Stack CoreS3
- PlatformIO 環境: `m5stack-cores3`
- 主な役割: BLE Beacon の広告送信
- 表示: 起動状態を簡易表示

### `m5`

M5Stack CoreS3 で動作する受信側プロトタイプです。BLE Beacon をスキャンして RSSI を取得し、歩数とあわせて SD カードへ CSV 保存します。

- 対象ボード: M5Stack CoreS3
- PlatformIO 環境: `m5stack-cores3`
- 主な役割: BLE 受信、歩数計測、CSV ログ保存
- 補足: `M5Unified` を使用しています

## PlatformIO の使い方

### 事前準備

1. VS Code を開く
2. PlatformIO IDE 拡張機能をインストールする
3. この `mvp` フォルダをワークスペースとして開く
4. 必要に応じてボードを USB 接続する

### 共通の設定

- `m5` はルートの共有ライブラリ `C:\Users\tugeu\dev\procon26\mvp\libraries` を参照します
- `ble_send` は共有ライブラリを使わず、レジストリ版の `M5Unified` と `M5GFX` を使います
- どちらも `post_build.py` を通してビルド後処理を実行します

### ビルド

各プロジェクトのフォルダでビルドします。

```powershell
python -m platformio run
```

### 書き込み

USB で接続した M5Stack に書き込みます。

```powershell
python -m platformio run --target upload
```

### シリアル監視

ログを確認するときはシリアルモニタを開きます。

```powershell
python -m platformio device monitor
```

## 動作イメージ

1. `ble_send` を起動して `M5_BEACON` を送信する
2. `m5` を起動して Beacon をスキャンする
3. `m5` 側で RSSI と歩数を取得し、SD カードに記録する

## 注意事項

- どちらも試作段階のため、動作保証や安定性は最終版ではありません。
- ハードウェアやファームウェアの変更に合わせて、ボード設定やライブラリ構成が変わる可能性があります。
- `m5` 側は SD カードを使うため、実機での確認が前提です。
- `ble_send` は CoreS3 向けに構成してありますが、ライブラリの更新でビルド結果が変わる可能性があります。

## 補足

- `ble_send` の PlatformIO 環境名は `m5stack-cores3`
- `m5` の PlatformIO 環境名も `m5stack-cores3`
