#include <M5Unified.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <BLEAdvertising.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <SD.h>
#include <time.h>
#include "cert.h"

#ifndef AUTO_UPDATE_SERVER_URL
#define AUTO_UPDATE_SERVER_URL "http://192.168.11.5:8000"
#endif

// ======================================================
// マルチスレッドの構成
//
// Core1（歩数と相対距離の取得）
//   ・loop()
//       - 歩数計算
//       - SDカード保存
//       - 画面描画
//       - 送信データ作成
//
//   ・BLE Task
//       - BLEスキャン
//       - devices[] 更新
//
// Core0（サーバとのデータ通信）
//   ・Send Task
//       - sendFlagを監視
//       - HTTP通信
//
// ======================================================

// ======================================================
// グローバル変数・マクロ変数の設定
// ======================================================

// 対応する児童の名前（デバイスごとに変える、NODE_のプレフィックスが必須）
// 名前の重複が無いように基本的にフルネームで登録することとする
#define DEVICE_ID "NODE_TESUTO"
#define AUTO_UPDATE_NAME_MAX_BYTES 24

#define CSV_BUFFER_SIZE 8192 // CSVバッファのサイズ
#define MAX_DISTANCE_COLUMNS 30 // CSVバッファのDistanceカラムの最大値
#define DEVICE_TIMEOUT_MS 10000 // 相対距離測定の際の相手デバイスのタイムアウト時間
#define UI_UPDATE_MS 10000 // 画面更新頻度
#define SCREEN_SLEEP_TIMEOUT_MS 30000 // 画面スリープまでの無操作時間 [ms] (30秒)

// 累計歩数のカウント
volatile int stepCount = 0;

// IMU（9軸: 加速度・ジャイロ・地磁気）
volatile float latestAx = 0, latestAy = 0, latestAz = 0;
volatile float latestGx = 0, latestGy = 0, latestGz = 0;
volatile float latestMx = 0, latestMy = 0, latestMz = 0;

// BLE（相対距離）
volatile int latestRSSI = -100;
volatile float distanceMeter = -1;

int distanceChildIds[MAX_DISTANCE_COLUMNS];
int distanceColumnCount = 0;

unsigned long csvSampleCount = 0;^
bool startWritten = false;

char measurementStartTime[32] = ""; // 計測開始時刻
char csvFileName[48] = "";

unsigned long lastCSVMillis = 0;
unsigned long lastFlushMillis = 0;
unsigned long lastSendMillis = 0;
unsigned long lastScreenTouchMillis = 0; // 最後に画面操作があった時刻
bool isScreenSleeping = false; // 画面スリープ状態（画面描画・バックライトOFF）

// Wi-Fi切断時に毎ループで再接続し、CPU負荷やログ出力が増えることを防ぐ
unsigned long lastWiFiReconnectMillis = 0;
unsigned int consecutiveCsvSendFailures = 0;
unsigned int consecutiveStatusSendFailures = 0;
// SD.begin() が失敗した場合、未マウントのままAPIを呼ばないようにする
bool sdCardMounted = false;

// センシング自体の周期はメインループのクロック周期に依存

const unsigned long CSV_INTERVAL = 100; // CSVバッファへの記録周期 [ms]
const unsigned long CSV_FLUSH_INTERVAL = 1000; // 計測データをSDへ書き込む周期 [ms]
const unsigned long SEND_INTERVAL = 10000; // 計測データのサーバへの送信周期 [ms]

// Wi-Fi切断中の再接続試行間隔 [ms]（画面・センシング処理を阻害しないよう間隔を空ける）
const unsigned long WIFI_RECONNECT_INTERVAL = 5000;

portMUX_TYPE deviceNameMux = portMUX_INITIALIZER_UNLOCKED;
char activeDeviceId[32] = DEVICE_ID;
BLEAdvertising* pAdvertising;

// ======================================================
// 児童ID・BLEデバイス管理
// ======================================================

// API_URLからスキーム+ホスト部分だけを取り出す関数（末尾のパスは含まない）
String getApiBaseUrl() {
    String url = API_URL;
    int pathPos = url.indexOf('/', 8);  // http(s):// の後の最初の /

    if (pathPos >= 0)
        url = url.substring(0, pathPos);

    return url;
}

String urlEncode(const String& value) {
    static const char hex[] = "0123456789ABCDEF";
    String encoded;
    encoded.reserve(value.length() * 3);

    for (size_t i = 0; i < value.length(); i++) {
        uint8_t byte = static_cast<uint8_t>(value[i]);
        if ((byte >= 'a' && byte <= 'z') ||
            (byte >= 'A' && byte <= 'Z') ||
            (byte >= '0' && byte <= '9') ||
            byte == '-' || byte == '_' || byte == '.' || byte == '~') {
            encoded += static_cast<char>(byte);
        } else {
            encoded += '%';
            encoded += hex[byte >> 4];
            encoded += hex[byte & 0x0F];
        }
    }
    return encoded;
}

void getActiveDeviceId(char* buffer, size_t bufferSize) {
    portENTER_CRITICAL(&deviceNameMux);
    strlcpy(buffer, activeDeviceId, bufferSize);
    portEXIT_CRITICAL(&deviceNameMux);
}

String getActiveDeviceId() {
    char deviceId[sizeof(activeDeviceId)];
    getActiveDeviceId(deviceId, sizeof(deviceId));
    return String(deviceId);
}

// 名札名をサーバーから取得し、BLE名を更新する
bool updateDeviceNameFromServer() {
    if (WiFi.status() != WL_CONNECTED) return false;

    String url = AUTO_UPDATE_SERVER_URL;
    while (url.endsWith("/")) url.remove(url.length() - 1);
    url += "/api/name";

    HTTPClient http;
    http.setTimeout(3000);
    if (!http.begin(url)) {
        Serial.println("Name update HTTP begin failed.");
        return false;
    }
    http.addHeader("Accept", "application/json");

    int responseCode = http.GET();
    if (responseCode != 200) {
        Serial.printf("Name update HTTP response: %d\n", responseCode);
        http.end();
        return false;
    }

    JsonDocument response;
    DeserializationError error = deserializeJson(response, http.getStream());
    http.end();
    if (error) {
        Serial.printf("Name update JSON parse failed: %s\n", error.c_str());
        return false;
    }

    const char* name = response["name"];
    if (name == nullptr || name[0] == '\0') return true;
    if (strlen(name) > AUTO_UPDATE_NAME_MAX_BYTES) {
        Serial.println("Name update ignored: name exceeds BLE advertising limit.");
        return false;
    }
    for (const unsigned char* character = (const unsigned char*)name; *character; character++) {
        if (*character < 32 || *character == 127) {
            Serial.println("Name update ignored: name contains control characters.");
            return false;
        }
    }

    String newDeviceId = "NODE_";
    newDeviceId += name;
    if (newDeviceId == getActiveDeviceId()) return true;

    portENTER_CRITICAL(&deviceNameMux);
    strlcpy(activeDeviceId, newDeviceId.c_str(), sizeof(activeDeviceId));
    portEXIT_CRITICAL(&deviceNameMux);

    BLEAdvertisementData advertisementData;
    advertisementData.setName(newDeviceId.c_str());
    pAdvertising->stop();
    pAdvertising->setAdvertisementData(advertisementData);
    pAdvertising->start();
    Serial.println("Device name updated from server.");
    return true;
}

// 児童名から対応するID値をDB検索して取得する関数
// デバイスに登録される児童名は重複が起きない想定で実装
int getChildId(const String& deviceName) {
    // NODE_のプレフィックスを除いて児童名（name）を取得
    String name = deviceName;
    if (name.startsWith("NODE_"))
        name = name.substring(5);

    // Wi-Fi接続に失敗した場合
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected.");
        return -1; // ID=-1として検索失敗扱いにする
    }

    // IDを取得するAPIのURLを作成する
    String url = getApiBaseUrl();
    url += "/api/children/search?name=" + urlEncode(name);

    Serial.println("Child ID search:");
    Serial.println("  name = " + name);
    Serial.println("  URL  = " + url);

    HTTPClient http;

    if (!http.begin(url)) {
        Serial.println("HTTP begin failed.");
        return -1;
    }

    http.addHeader("Accept", "application/json");

    int httpResponseCode = http.GET();

    if (httpResponseCode <= 0) {
        Serial.print("HTTP GET error: ");
        Serial.println(httpResponseCode);
        http.end();
        return -1;
    }

    Serial.print("HTTP Response: ");
    Serial.println(httpResponseCode);

    String response = http.getString();
    Serial.println("Response: " + response);

    http.end();

    if (httpResponseCode != 200) {
        Serial.println("Child search failed.");
        return -1;
    }

    // JSONを解析する
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, response);

    if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return -1;
    }

    // ID値を取得
    if (!doc.containsKey("child_id")) {
        // DBに対応するID値が見つからなかった場合
        Serial.println("child_id not found in response.");
        return -1;
    }

    int childId = doc["child_id"];

    Serial.print("Found child_id: ");
    Serial.println(childId);

    return childId; // 児童名に対応するID値を返す
}

// 指定したID値が現在存在しているかどうかを確認する関数
bool hasDistanceColumn(int childId) {
    for (int i = 0; i < distanceColumnCount; i++) {
        if (distanceChildIds[i] == childId) return true;
    }
    return false;
}

// 信号強度からの相対距離計算用の関数
float calculateDistance(int rssi) {
    int txPower = -59;

    if (rssi == 0)
        return -1;

    float ratio = rssi * 1.0 / txPower;

    if (ratio < 1.0) {
        return pow(ratio, 10);
    }

    return 0.89976 * pow(ratio, 7.7095) + 0.111; // 信号強度計算
}

BLEScan* pBLEScan;
TaskHandle_t bleTaskHandle;
TaskHandle_t sendTaskHandle;
portMUX_TYPE sharedStateMux = portMUX_INITIALIZER_UNLOCKED;

// 一定間隔でサーバへCSVを送信するためのトリガー
volatile bool sendFlag = false;

// 複数ノードの管理
struct DeviceInfo {
    char id[32]; // Stringにすると更新時にヒープ確保が走り、クリティカルセクション内でのmallocは危険なためchar配列にする
    int childId;
    int rssi;
    float distance;
    unsigned long lastSeen;
};

DeviceInfo devices[20];
int deviceCount = 0;

// ノード更新
void updateDevice(const char* id, int rssi) {
    float distance = calculateDistance(rssi);

    for (int i = 0; i < deviceCount; i++) {
        if (strcmp(devices[i].id, id) == 0) {
            devices[i].rssi = rssi;
            devices[i].distance = distance;
            devices[i].lastSeen = millis();

            return;
        }
    }

    if (deviceCount < 20) {
        strlcpy(devices[deviceCount].id, id, sizeof(devices[deviceCount].id));
        devices[deviceCount].childId = -1;
        devices[deviceCount].rssi = rssi;
        devices[deviceCount].distance = distance;
        devices[deviceCount].lastSeen = millis();

        deviceCount++;
    }
}

// BLE callback
class MyCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) {
        String name = device.getName().c_str();

        if (!name.startsWith("NODE") || name == getActiveDeviceId())
            return;

        int rssi = device.getRSSI();
        float distance = calculateDistance(rssi);

        portENTER_CRITICAL(&sharedStateMux); // devices[]への同時アクセス回避

        updateDevice(name.c_str(), rssi);

        latestRSSI = rssi;
        distanceMeter = distance;

        portEXIT_CRITICAL(&sharedStateMux); // devices[]への同時アクセス回避

        Serial.printf("%s RSSI=%d Dist=%.2f\n", name.c_str(), rssi, distance);
    }
};

// BLE通信用スレッド
void bleTask(void *arg) {
    while (true) {
        pBLEScan->start(1, false);
        pBLEScan->clearResults();

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// ======================================================
// センシング・歩数計算
// ======================================================

// 値をセンシングして最新値を更新する
void updateSensors() {
    float ax, ay, az, gx, gy, gz, mx, my, mz;

    M5.Imu.getAccelData(&ax, &ay, &az);
    M5.Imu.getGyroData(&gx, &gy, &gz);
    M5.Imu.getMag(&mx, &my, &mz);

    latestAx = ax; latestAy = ay; latestAz = az;
    latestGx = gx; latestGy = gy; latestGz = gz;
    latestMx = mx; latestMy = my; latestMz = mz;
}

// 加速度からの歩数計算用関数
void updateStepCount() {
    static float gravity = 1.0f;
    static float filtered = 0.0f;
    static float prevFiltered = 0.0f;
    static bool rising = false;
    static unsigned long lastStepMillis = 0;
    static int calibrationCount = 0;

    const float ALPHA = 0.92f;
    const float STEP_THRESHOLD = 0.18f;
    const unsigned long STEP_INTERVAL = 300;

    float accelMagnitude = sqrt(
        latestAx * latestAx +
        latestAy * latestAy +
        latestAz * latestAz
    );

    // 起動直後の重力加速度を簡易キャリブレーション
    if (calibrationCount < 40) {
        gravity = gravity * 0.9f + accelMagnitude * 0.1f;
        calibrationCount++;
        return;
    }

    gravity = gravity * ALPHA + accelMagnitude * (1.0f - ALPHA);
    filtered = accelMagnitude - gravity;

    bool currentRising = filtered > prevFiltered;

    if (rising && !currentRising && prevFiltered > STEP_THRESHOLD) {
        unsigned long now = millis();

        if (now - lastStepMillis > STEP_INTERVAL) {
            stepCount++;
            lastStepMillis = now;
            Serial.printf("STEP %d\n", stepCount);
        }
    }

    rising = currentRising;
    prevFiltered = filtered;
}

// ======================================================
// CSVデータの作成・SDカードへの保存
// ======================================================

// CSV形式の文字列を格納する変数（固定サイズ、ヒープ確保なし）
// SDカードへの書き込み回数を減らすためのバッファとなる
char csvBuffer[CSV_BUFFER_SIZE];
size_t csvBufferLen = 0;

// サーバへpush_csvするまでの間、行を貯めておくバッファ（SD用とは別に独立して管理）
char csvSendBuffer[CSV_BUFFER_SIZE];
size_t csvSendBufferLen = 0;
portMUX_TYPE csvSendBufferMux = portMUX_INITIALIZER_UNLOCKED;

// push_csvは送信したCSVの先頭行のStart列から計測開始時刻を読み取る仕様のため、
// SD保存用のstartWrittenとは別に、送信バッチの先頭行かどうかを管理する
bool sendBatchStartWritten = false;

// SDカードの初期化処理
void initSDCard() {
    // 初期化結果を保持し、以降のCSV処理で未マウントのAPI呼び出しを防ぐ
    sdCardMounted = SD.begin(4);
    if (!sdCardMounted) {
        Serial.println("SD card initialization failed");
        return;
    }

    Serial.println("SD card initialized");
}

// インターネット非接続環境向け: NTPは使わず、システム時刻を固定値
// 2026-10-10 09:30:00 JST に設定する
void setFixedTime() {
    setenv("TZ", "JST-9", 1);
    tzset();

    struct tm t = {};
    t.tm_year = 2026 - 1900;
    t.tm_mon = 10 - 1;
    t.tm_mday = 10;
    t.tm_hour = 9;
    t.tm_min = 30;
    t.tm_sec = 0;
    t.tm_isdst = 0;

    struct timeval tv = {mktime(&t), 0};
    settimeofday(&tv, nullptr);
}

// 計測開始時刻を取得する関数
void getStartTime(char* buffer, size_t size) {
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);

    // configTime()で設定したタイムゾーン（JST）の計測開始時刻を取得
    snprintf(buffer, size, "%04d-%02d-%02d %02d:%02d:%02d",
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
            t->tm_hour, t->tm_min, t->tm_sec);
}

// CSVバッファを初期化する関数（最初に一度呼ばれる）
void createNewCSVFile() {
    // 計測開始時刻はSDの有無に関わらずサーバ送信にも必要
    getStartTime(measurementStartTime, sizeof(measurementStartTime));
    distanceColumnCount = 0;
    csvSampleCount = 0;
    startWritten = false;

    // SD未装着・初期化失敗時は計測とネットワーク送信を継続し、CSVの保存処理だけ停止
    if (!sdCardMounted) return;

    time_t now = time(nullptr);
    struct tm* t = localtime(&now);

    snprintf(csvFileName, sizeof(csvFileName), "/data_%04d%02d%02d_%02d%02d%02d.csv",
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
            t->tm_hour, t->tm_min, t->tm_sec);

    // 計測開始時刻はファイル名と同じ時刻
    File file = SD.open(csvFileName, FILE_WRITE);
    if (!file) {
        Serial.println("CSV file creation failed.");
        return;
    }

    // 最初に各カラムを用意しておく
    file.println("Timestamp,Steps,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Start");
    file.close();

    Serial.print("CSV created: ");
    Serial.println(csvFileName);
    Serial.print("Start: ");
    Serial.println(measurementStartTime);

    // 最初は相対距離用のカラムを作らない
}

// 引数の新たな行をCSVバッファへ追記する関数
void appendToCSVBuffer(const char* line) {
    size_t lineLen = strlen(line);

    if (csvBufferLen + lineLen >= CSV_BUFFER_SIZE)
        flushCSVBuffer();

    memcpy(csvBuffer + csvBufferLen, line, lineLen);
    csvBufferLen += lineLen;
}

// 引数の新たな行をサーバ送信用バッファへ追記する関数
// 送信が間に合わずバッファが満杯の場合は、次の送信まで新しい行を取りこぼす
void appendToSendBuffer(const char* prefix, const char* distanceSuffix) {
    size_t prefixLen = strlen(prefix);
    size_t suffixLen = strlen(distanceSuffix);

    portENTER_CRITICAL(&csvSendBufferMux);
    size_t startLen = sendBatchStartWritten ? 0 : strlen(measurementStartTime);
    size_t lineLen = prefixLen + startLen + suffixLen + 1;
    if (csvSendBufferLen + lineLen >= CSV_BUFFER_SIZE) {
        portEXIT_CRITICAL(&csvSendBufferMux);
        Serial.println("CSV send buffer is full; dropping a row.");
        return;
    }

    memcpy(csvSendBuffer + csvSendBufferLen, prefix, prefixLen);
    csvSendBufferLen += prefixLen;
    if (startLen > 0) {
        memcpy(csvSendBuffer + csvSendBufferLen, measurementStartTime, startLen);
        csvSendBufferLen += startLen;
    }
    memcpy(csvSendBuffer + csvSendBufferLen, distanceSuffix, suffixLen);
    csvSendBufferLen += suffixLen;
    csvSendBuffer[csvSendBufferLen++] = '\n';
    sendBatchStartWritten = true;
    portEXIT_CRITICAL(&csvSendBufferMux);
}

// 送信成功時に送信済み行だけを除き、残ったバッチの先頭行にStart時刻を戻す。
void ensureFirstSendRowHasStartLocked() {
    if (csvSendBufferLen == 0) {
        sendBatchStartWritten = false;
        return;
    }

    size_t commaCount = 0;
    size_t insertPosition = 0;
    for (size_t i = 0; i < csvSendBufferLen && csvSendBuffer[i] != '\n'; i++) {
        if (csvSendBuffer[i] == ',' && ++commaCount == 11) {
            insertPosition = i + 1;
            break;
        }
    }

    if (insertPosition > 0 &&
        (csvSendBuffer[insertPosition] == ',' || csvSendBuffer[insertPosition] == '\n')) {
        size_t startLen = strlen(measurementStartTime);
        if (csvSendBufferLen + startLen < CSV_BUFFER_SIZE) {
            memmove(csvSendBuffer + insertPosition + startLen,
                    csvSendBuffer + insertPosition,
                    csvSendBufferLen - insertPosition);
            memcpy(csvSendBuffer + insertPosition, measurementStartTime, startLen);
            csvSendBufferLen += startLen;
        }
    }
    sendBatchStartWritten = true;
}

// 送信待ちの行があるかを確認する。
bool hasPendingSendData() {
    portENTER_CRITICAL(&csvSendBufferMux);
    bool hasData = csvSendBufferLen > 0;
    portEXIT_CRITICAL(&csvSendBufferMux);
    return hasData;
}

// 送信バッファ内の既存行にもDistance列の空欄を追加する
bool appendDistanceColumnToSendBuffer() {
    size_t rowCount = 0;

    for (size_t i = 0; i < csvSendBufferLen; i++) {
        if (csvSendBuffer[i] == '\n') rowCount++;
    }

    if (csvSendBufferLen + rowCount >= CSV_BUFFER_SIZE) return false;

    size_t writePos = csvSendBufferLen + rowCount;
    for (size_t readPos = csvSendBufferLen; readPos > 0; readPos--) {
        char value = csvSendBuffer[readPos - 1];
        csvSendBuffer[--writePos] = value;
        if (value == '\n') csvSendBuffer[--writePos] = ',';
    }

    csvSendBufferLen += rowCount;
    return true;
}

// Distance列を後から追加する関数（検出されたデバイス数に対応）
// 引数に渡したID値の児童用の相対距離のカラムを新しくCSVに追加する
bool addDistanceColumn(int childId) {
    // 既にその児童用の相対距離のカラムがある場合
    if (hasDistanceColumn(childId)) return true;

    // 作成するDistance列の数が上限に達している場合
    if (distanceColumnCount >= MAX_DISTANCE_COLUMNS) return false;

    // SDが使えない場合も、サーバ送信用の列構成は維持する
    if (!sdCardMounted) {
        portENTER_CRITICAL(&csvSendBufferMux);
        bool appended = appendDistanceColumnToSendBuffer();
        if (appended) {
            distanceChildIds[distanceColumnCount] = childId;
            distanceColumnCount++;
        }
        portEXIT_CRITICAL(&csvSendBufferMux);
        if (!appended) return false;
        return true;
    }

    flushCSVBuffer(); // 一度SDカードにはこの時点のCSVバッファを書き込んでおく

    // ======================================================
    // 一度tmpを作ってそこに新たに必要となったDistance列を追加
    // 作成したtmpを元のCSVバッファに置き換えるという実装
    // 既にあるDistance列は削除しないで全て残しておく
    // ======================================================

    char tempFileName[64];
    snprintf(tempFileName, sizeof(tempFileName), "%s.tmp", csvFileName);

    // 前回のtmpファイルが残っていたら削除する
    if (SD.exists(tempFileName)) SD.remove(tempFileName);

    File src = SD.open(csvFileName, FILE_READ);
    File dst = SD.open(tempFileName, FILE_WRITE);

    if (!src || !dst) {
        if (src) src.close();
        if (dst) dst.close();
        Serial.println("Failed to remove temporary CSV.");
        return false;
    }

    char line[512];

    // ヘッダーに新しいDistance列を追加する
    if (src.available()) {
        size_t len = src.readBytesUntil('\n', line, sizeof(line) - 1);
        line[len] = '\0';

        while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n'))
            line[--len] = '\0';

        dst.print(line);
        dst.print(",Distance_");
        dst.println(childId);
    }

    // 既存データの末尾に空欄を1つ追加（改行文字を取り除く）
    while (src.available()) {
        size_t len = src.readBytesUntil('\n', line, sizeof(line) - 1);
        line[len] = '\0';

        while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n'))
            line[--len] = '\0';

        dst.print(line);
        dst.println(",");
    }

    src.close();
    dst.close();

    // tmpを元のCSVバッファと置き換える（元のCSVバッファは削除）
    if (!SD.remove(csvFileName)) {
        Serial.println("Failed to remove old CSV.");
        SD.remove(tempFileName);
        return false;
    }

    // tmpをリネームしておく
    if (!SD.rename(tempFileName, csvFileName)) {
        Serial.println("Failed to rename temporary CSV.");
        return false;
    }

    portENTER_CRITICAL(&csvSendBufferMux);
    bool appended = appendDistanceColumnToSendBuffer();
    if (appended) {
        distanceChildIds[distanceColumnCount] = childId;
        distanceColumnCount++;
    }
    portEXIT_CRITICAL(&csvSendBufferMux);
    if (!appended) return false;

    Serial.print("Added Distance_");
    Serial.println(childId);

    return true;
}

// 新たに検出された児童がいるかどうかを確認する関数
void updateDistanceColumns() {
    DeviceInfo snapshot[20];
    int count;

    portENTER_CRITICAL(&sharedStateMux);  // devices[]への同時アクセス回避
    count = deviceCount;
    if (count > 20) count = 20;

    for (int i = 0; i < count; i++)
        snapshot[i] = devices[i];

    portEXIT_CRITICAL(&sharedStateMux);  // devices[]への同時アクセス回避

    for (int i = 0; i < count; i++) {
        // タイムアウトしたデバイスは現在の相対距離の測定対象から除外
        if (millis() - snapshot[i].lastSeen > DEVICE_TIMEOUT_MS) continue;

        int childId = snapshot[i].childId;

        // 新たに検出されたデバイス（児童）がいる場合
        if (childId > 0 && !hasDistanceColumn(childId)) {
            // 新たなDistance列を追加する
            if (!addDistanceColumn(childId)) {
                Serial.print("Distance_");
                Serial.print(childId);
                Serial.println(" の追加に失敗しました。");
            }
        }
    }
}

// 現時点での計測データからCSVバッファ（SD保存用・サーバ送信用）を更新する関数
void saveDataToCSV() {
    updateDistanceColumns(); // 現状の相対距離測定の相手デバイスを確認する（SD未マウント時も送信用バッファに列追加が反映される）

    // ======================================================
    // この時点で必要な分のCSVのカラムは用意は完了済み
    // 元々あったDistance列はタイムアウトしていても残されている
    // ======================================================

    DeviceInfo snapshot[20];
    int count;

    portENTER_CRITICAL(&sharedStateMux);  // devices[]への同時アクセス回避
    count = deviceCount;
    if (count > 20) count = 20;

    for (int i = 0; i < count; i++) {
        snapshot[i] = devices[i];
    }
    portEXIT_CRITICAL(&sharedStateMux);  // devices[]への同時アクセス回避

    // Timestamp〜Mzまでの列はSD保存用・サーバ送信用で共通
    String prefix;
    prefix.reserve(256);

    // Timestampは0.0, 0.1, 0.2...とサンプル番号から生成する
    float timestamp = csvSampleCount * 0.1f;

    prefix += String(timestamp, 1);
    prefix += ",";
    prefix += String(stepCount);
    prefix += ",";
    prefix += String(latestAx, 4);
    prefix += ",";
    prefix += String(latestAy, 4);
    prefix += ",";
    prefix += String(latestAz, 4);
    prefix += ",";
    prefix += String(latestGx, 4);
    prefix += ",";
    prefix += String(latestGy, 4);
    prefix += ",";
    prefix += String(latestGz, 4);
    prefix += ",";
    prefix += String(latestMx, 4);
    prefix += ",";
    prefix += String(latestMy, 4);
    prefix += ",";
    prefix += String(latestMz, 4);
    prefix += ",";

    // 登録済みのDistance列をすべて出力（SD保存用・サーバ送信用で共通）
    String distanceSuffix;
    for (int c = 0; c < distanceColumnCount; c++) {
        distanceSuffix += ",";

        for (int i = 0; i < count; i++) {
            if (millis() - snapshot[i].lastSeen > DEVICE_TIMEOUT_MS) continue;

            if (snapshot[i].childId == distanceChildIds[c]) {
                distanceSuffix += String(snapshot[i].distance, 2);
                break;
            }
        }

        // 見つからない場合は空欄のまま
    }

    // SDカードへの保存用の行（Start＝計測開始時刻は計測全体で最初の1行だけに書く）
    if (sdCardMounted) {
        String sdRow = prefix;

        if (!startWritten) {
            sdRow += measurementStartTime;
            startWritten = true;
        }

        sdRow += distanceSuffix;
        sdRow += "\n";

        appendToCSVBuffer(sdRow.c_str());
    }

    // Start列の有無は送信バッファのロック中に決め、送信中の更新と競合させない
    appendToSendBuffer(prefix.c_str(), distanceSuffix.c_str());

    csvSampleCount++;
}

// 現時点でのCSVバッファをSDカードへ書き込む関数（一定間隔でまとめて書き込む）
void flushCSVBuffer() {
    // SD未マウント時のopen失敗とエラーログの連続出力を防ぐ
    if (!sdCardMounted || csvFileName[0] == '\0' || csvBufferLen == 0) return;

    File file = SD.open(csvFileName, FILE_APPEND);

    if (file) {
        file.write((const uint8_t*)csvBuffer, csvBufferLen);
        file.close();
    }

    csvBufferLen = 0; // 書き込んだ後は長さを0に戻す
}

// ======================================================
// サーバとの通信処理
// ======================================================

// Wi-Fiの再接続を行う関数
void reconnectWiFiIfNeeded() {
    if (WiFi.status() == WL_CONNECTED) return;

    unsigned long now = millis();
    if (now - lastWiFiReconnectMillis < WIFI_RECONNECT_INTERVAL) return;

    lastWiFiReconnectMillis = now;
    Serial.println("WiFi reconnecting...");

    // 再接続は通信タスクで行い、loopの画面更新をブロックしない
    WiFi.reconnect();
}

// 同じ種類の送信が2回続けて失敗したらWi-Fi接続を張り直す。
void recordServerSendResult(bool succeeded, unsigned int& consecutiveFailures, const char* dataType) {
    if (succeeded) {
        consecutiveFailures = 0;
        return;
    }

    consecutiveFailures++;
    Serial.println(String(dataType) + " send failed (" + consecutiveFailures + "/2)");
    if (consecutiveFailures < 2) return;

    consecutiveCsvSendFailures = 0;
    consecutiveStatusSendFailures = 0;
    Serial.println("Repeated server send failures; restarting WiFi connection...");
    WiFi.disconnect(false, false);
    lastWiFiReconnectMillis = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// センシングをサーバ側と同期させる関数
void resolveDeviceChildIds() {
    DeviceInfo snapshot[20];
    int count;

    portENTER_CRITICAL(&sharedStateMux);  // devices[]への同時アクセス回避

    count = min(deviceCount, 20);
    for (int i = 0; i < count; i++) snapshot[i] = devices[i];

    portEXIT_CRITICAL(&sharedStateMux);  // devices[]への同時アクセス回避

    for (int i = 0; i < count; i++) {
        if (snapshot[i].childId > 0 || millis() - snapshot[i].lastSeen > DEVICE_TIMEOUT_MS)
            continue;

        // HTTP検索は通信タスクで実行し、CSV/UI更新周期にはネットワーク待ちを持ち込まない
        int childId = getChildId(snapshot[i].id);

        // childIdが負の値であった場合
        if (childId <= 0) continue;

        portENTER_CRITICAL(&sharedStateMux);  // devices[]への同時アクセス回避
        for (int j = 0; j < deviceCount; j++) {
            if (strcmp(devices[j].id, snapshot[i].id) == 0) {
                devices[j].childId = childId;
                break;
            }
        }
        portEXIT_CRITICAL(&sharedStateMux);  // devices[]への同時アクセス回避
    }
}

// サーバへ蓄積したCSV行をpush_csvとして送信する関数
void sendCSVBufferToServer(int ownChildId) {
    if (!hasPendingSendData()) return;

    // Wi-Fiのコネクションを確認
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected!");
        recordServerSendResult(false, consecutiveCsvSendFailures, "CSV");
        return;
    }

    if (ownChildId <= 0) {
        Serial.println("CSV send skipped because child ID is unavailable.");
        recordServerSendResult(false, consecutiveCsvSendFailures, "CSV");
        return;
    }

    uint8_t* body = (uint8_t*)malloc(CSV_BUFFER_SIZE + 1024);
    if (!body) {
        Serial.println("CSV送信用バッファのmallocに失敗しました。");
        recordServerSendResult(false, consecutiveCsvSendFailures, "CSV");
        return;
    }

    size_t batchLen = 0;
    size_t batchRowCount = 0;
    int batchDistanceColumnCount = 0;
    int batchDistanceChildIds[MAX_DISTANCE_COLUMNS];
    portENTER_CRITICAL(&csvSendBufferMux);
    batchLen = csvSendBufferLen;
    if (batchLen > 0) {
        memcpy(body, csvSendBuffer, batchLen);
        for (size_t i = 0; i < batchLen; i++) {
            if (csvSendBuffer[i] == '\n') batchRowCount++;
        }
    }
    batchDistanceColumnCount = distanceColumnCount;
    memcpy(batchDistanceChildIds, distanceChildIds,
           batchDistanceColumnCount * sizeof(batchDistanceChildIds[0]));
    portEXIT_CRITICAL(&csvSendBufferMux);

    if (batchLen == 0) {
        free(body);
        return;
    }

    // push_csvは1行目をヘッダーとして読み飛ばす仕様のため、現在の列構成を付けて送信する
    String csvHeader = "Timestamp,Steps,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Start";
    for (int i = 0; i < batchDistanceColumnCount; i++) {
        csvHeader += ",Distance_";
        csvHeader += String(batchDistanceChildIds[i]);
    }
    csvHeader += "\n";

    size_t headerLen = csvHeader.length();
    size_t totalLen = headerLen + batchLen;
    if (totalLen > CSV_BUFFER_SIZE + 1024) {
        free(body);
        recordServerSendResult(false, consecutiveCsvSendFailures, "CSV");
        return;
    }
    memmove(body + headerLen, body, batchLen);
    memcpy(body, csvHeader.c_str(), headerLen);

    String url = getApiBaseUrl();
    url += "/api/push_csv/" + String(ownChildId);

    HTTPClient http;
    if (!http.begin(url)) {
        Serial.println("CSV HTTP begin failed.");
        free(body);
        recordServerSendResult(false, consecutiveCsvSendFailures, "CSV");
        return;
    }
    http.addHeader("Content-Type", "text/csv");

    int httpResponseCode = http.POST(body, totalLen);

    free(body);

    if (httpResponseCode > 0) {
        Serial.println("HTTP Response: " + String(httpResponseCode));
        String response = http.getString();
        Serial.println("Response: " + response);

        if (httpResponseCode == 200) {
            // POST中に追加された行を残し、送信開始時点の行だけを取り除く
            portENTER_CRITICAL(&csvSendBufferMux);
            size_t removeBytes = 0;
            size_t removedRows = 0;
            while (removeBytes < csvSendBufferLen && removedRows < batchRowCount) {
                if (csvSendBuffer[removeBytes++] == '\n') removedRows++;
            }
            if (removedRows == batchRowCount && removeBytes > 0) {
                memmove(csvSendBuffer, csvSendBuffer + removeBytes,
                        csvSendBufferLen - removeBytes);
                csvSendBufferLen -= removeBytes;
                ensureFirstSendRowHasStartLocked();
            }
            portEXIT_CRITICAL(&csvSendBufferMux);
        }
    } else {
        Serial.println("Error: " + String(httpResponseCode));
    }

    http.end();
    recordServerSendResult(httpResponseCode == 200, consecutiveCsvSendFailures, "CSV");
}

// バッテリー残量(%)を取得する関数
// 12%付近で電源が切れてしまうため、12%〜100%を0%〜100%に補正する
int getBatteryLevel() {
    int raw = M5.Power.getBatteryLevel();
    if (raw < 0 || raw > 100) {
        return raw;
    }
    const int BATTERY_CUTOFF_PERCENT = 12;
    if (raw <= BATTERY_CUTOFF_PERCENT) {
        return 0;
    }
    int adjusted = (int)round((raw - BATTERY_CUTOFF_PERCENT) * 100.0 / (100.0 - BATTERY_CUTOFF_PERCENT));
    return constrain(adjusted, 0, 100);
}

// M5のバッテリー残量とWi-Fi RSSIをサーバへ送信する関数
void sendDeviceStatusToServer(int ownChildId) {
    int battery = getBatteryLevel();
    if (battery < 0 || battery > 100) {
        Serial.println("Invalid battery level; device status was not sent.");
        recordServerSendResult(false, consecutiveStatusSendFailures, "Device status");
        return;
    }

    StaticJsonDocument<128> doc;
    doc["child_id"] = ownChildId;
    doc["battery"] = battery;
    doc["wifi_rssi"] = WiFi.RSSI();

    String body;
    serializeJson(doc, body);

    String url = getApiBaseUrl();
    url += "/api/device_status";

    HTTPClient http;
    if (!http.begin(url)) {
        Serial.println("Device status HTTP begin failed.");
        recordServerSendResult(false, consecutiveStatusSendFailures, "Device status");
        return;
    }

    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(body);
    if (httpResponseCode > 0) {
        Serial.println("Device status HTTP Response: " + String(httpResponseCode));
    } else {
        Serial.println("Device status HTTP error: " + String(httpResponseCode));
    }
    http.end();
    recordServerSendResult(httpResponseCode == 200, consecutiveStatusSendFailures, "Device status");
}

// サーバとの通信用スレッド
void sendTask(void *arg)
{
    while (true) {
        reconnectWiFiIfNeeded(); // 必要ならWi-Fiの再接続

        if (sendFlag) {
            portENTER_CRITICAL(&sharedStateMux); // devices[]への同時アクセス回避
            sendFlag = false;
            portEXIT_CRITICAL(&sharedStateMux); // devices[]への同時アクセス回避

            int ownChildId = -1;
            bool hasCsvData = hasPendingSendData();
            if (WiFi.status() == WL_CONNECTED) {
                resolveDeviceChildIds(); // Wi-Fi接続がされていたらサーバ側と同期させる
                ownChildId = getChildId(getActiveDeviceId());
                if (ownChildId > 0) {
                    sendDeviceStatusToServer(ownChildId);
                } else if (!hasCsvData) {
                    recordServerSendResult(false, consecutiveStatusSendFailures, "Device status");
                }
            } else if (!hasCsvData) {
                recordServerSendResult(false, consecutiveStatusSendFailures, "Device status");
            }

            sendCSVBufferToServer(ownChildId);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// ======================================================
// UI描画処理
// ======================================================

int oldStepUI = -1;
int oldBatteryUI = -1;

// UI初期描画
void drawUIBase() {
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(WHITE);
    M5.Display.setBrightness(40); //画面明るさ

    // デバイス名
    M5.Display.setTextSize(5);
    M5.Display.setCursor(20, 20);
    String deviceId = getActiveDeviceId();
    M5.Display.println(deviceId.startsWith("NODE_") ? deviceId.substring(5) : deviceId);

    // STEPのラベル
    M5.Display.setTextSize(3);
    M5.Display.setCursor(20, 90);
    M5.Display.printf("STEP:");
}

// STEP部分のUI更新
void updateStepUI() {
    // 表示する値が変わっていない場合は更新する必要がない
    if (oldStepUI == stepCount)
        return;

    oldStepUI = stepCount; // 以前の値を覚えておく

    // 数字だけ消す
    M5.Display.fillRect(120, 90, 120, 30, BLACK);

    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(3);
    M5.Display.setCursor(120, 90);
    M5.Display.printf("%d", stepCount);
}

// 相対距離の表示のUI更新
void updateDistanceUI() {
    const int listX = 20;
    const int listY = 140;
    const int listW = 300;
    const int listH = 60;
    
    M5.Display.fillRect(listX, listY, listW, listH, ILI9341_BLACK);

    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);

    int y = listY;

    portENTER_CRITICAL(&sharedStateMux); // devices[]への同時アクセス回避

    for (int i = 0; i < deviceCount; i++) {
        if (millis() - devices[i].lastSeen > 10000)
            continue;

        // BATエリアに文字が被らないようにする
        if (y + 25 > listY + listH)
            break;

        M5.Display.setCursor(listX, y);

        // NODE_の部分は削って相対距離の相手の名前は表示させる
        const char* displayName = devices[i].id;
        if (strncmp(displayName, "NODE_", 5) == 0) displayName += 5;
        M5.Display.printf("%s %.1fm", displayName, devices[i].distance);

        y += 25;
    }

    portEXIT_CRITICAL(&sharedStateMux); // devices[]への同時アクセス回避    
}

// Battery部分のUI更新
void updateBatteryUI() {
    int battery = getBatteryLevel();

    // 取得失敗（-1などの無効値）は無視して、前回表示を維持する
    if (battery < 0)
        return;
    
    // 表示する値が変わっていない場合は更新する必要がない
    if (battery == oldBatteryUI)
        return;

    oldBatteryUI = battery; // 以前の値を覚えておく

    const int x = M5.Display.width() - 160;
    const int y = M5.Display.height() - 30;
    const int w = 160;
    const int h = 30;

    M5.Display.fillRect(x, y, w, h, BLACK);

    M5.Display.setTextColor(TFT_YELLOW);
    M5.Display.setTextSize(3);
    M5.Display.setCursor(x, y);
    M5.Display.printf("BAT:%d%%", battery);
}

// UI描画（部分の更新のみ）
void drawUI() {
    updateStepUI();
    updateDistanceUI();
    updateBatteryUI();
}

// 画面スリープ（バックライト消灯・描画停止）
void sleepScreen() {
    if (isScreenSleeping)
        return;
    isScreenSleeping = true;
    M5.Display.sleep();
    M5.Display.setBrightness(0);
    Serial.println("Screen sleep activated.");
}

// 画面復帰（バックライト点灯・UI再描画）
void wakeupScreen() {
    if (!isScreenSleeping)
        return;
    isScreenSleeping = false;
    M5.Display.wakeup();
    M5.Display.setBrightness(40);
    drawUIBase();
    oldStepUI = -1;
    oldBatteryUI = -1;
    drawUI();
    Serial.println("Screen wakeup activated.");
}

// ======================================================
// デバイスのセットアップ
// ======================================================

// 初期化処理
void setup() {
    auto cfg = M5.config();

    cfg.serial_baudrate = 115200;
    cfg.clear_display = true;

    M5.begin(cfg);

    Serial.begin(115200);

    delay(100);

    M5.Display.setRotation(1);

    initSDCard();

    BLEDevice::init(DEVICE_ID);

    pAdvertising = BLEDevice::getAdvertising();

    BLEAdvertisementData advData;

    advData.setName(DEVICE_ID);

    pAdvertising->setAdvertisementData(advData);

    pAdvertising->start();
    
    pBLEScan = BLEDevice::getScan(); // Scan

    pBLEScan->setAdvertisedDeviceCallbacks(new MyCallbacks());

    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(80);

    xTaskCreatePinnedToCore(bleTask, "BLE", 4096, NULL, 2, &bleTaskHandle, 1);
    xTaskCreatePinnedToCore(sendTask, "SEND", 8192, NULL, 1, &sendTaskHandle, 0);

    setFixedTime(); // NTPなしで計測開始時刻を固定

    // Wi-Fiへの接続を行う
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int dotCount = 0;
    const int WIFI_CONNECT_DOT_INTERVAL_MS = 500; // ドット表示間隔 [ms]
    const int WIFI_CONNECT_TIMEOUT_DOTS = 20000000; // タイムアウトまでのドット数

    while (WiFi.status() != WL_CONNECTED && dotCount < WIFI_CONNECT_TIMEOUT_DOTS) {
        delay(WIFI_CONNECT_DOT_INTERVAL_MS); // Wi-Fi接続試行中にはドットを表示していく
        M5.Lcd.print(".");
        dotCount++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.println("WiFi connected"); // Wi-Fi接続完了
        M5.Lcd.print("IP address = ");
        M5.Lcd.println(WiFi.localIP()); // デバイスのローカルIPアドレスを表示

        updateDeviceNameFromServer(); // Wi-Fi接続後に名札名を同期
    } else {
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.println("WiFi Timeout"); // Wi-Fi接続タイムアウト
    }

    createNewCSVFile(); // 同期できていればその時刻でファイル名を生成する
    lastCSVMillis = millis();

    drawUIBase(); // UIの初期描画
    updateBatteryUI(); // 初回のバッテリー残量を表示

    lastScreenTouchMillis = millis();
    isScreenSleeping = false;
}

// ======================================================
// メインループ
// ======================================================

unsigned long lastUI = 0; // 画面更新頻度のパラメータ

// メインループ処理
void loop() {
    M5.update(); // タッチ・ボタン状態の更新

    unsigned long now = millis();

    // 画面タップ・操作の検知（CoreS3画面タッチ・仮想ボタン）
    bool isTouched = (M5.Touch.getCount() > 0) ||
                     M5.Touch.getDetail().wasClicked() ||
                     M5.Touch.getDetail().wasPressed() ||
                     M5.BtnPWR.wasClicked() ||
                     M5.BtnA.wasPressed() ||
                     M5.BtnB.wasPressed() ||
                     M5.BtnC.wasPressed();

    if (isTouched) {
        if (isScreenSleeping) {
            wakeupScreen();
        }
        lastScreenTouchMillis = now;
    }

    // 30秒間操作がなければ画面をスリープ
    if (!isScreenSleeping && (now - lastScreenTouchMillis >= SCREEN_SLEEP_TIMEOUT_MS)) {
        sleepScreen();
    }

    updateSensors(); // センシングした値を更新
    updateStepCount(); // 最新のセンシング値から歩数を計算して更新

    static char lastDisplayedDeviceId[sizeof(activeDeviceId)] = DEVICE_ID;
    char currentDeviceId[sizeof(activeDeviceId)];
    getActiveDeviceId(currentDeviceId, sizeof(currentDeviceId));
    if (strcmp(lastDisplayedDeviceId, currentDeviceId) != 0) {
        if (!isScreenSleeping) {
            drawUIBase();
            oldStepUI = -1;
            oldBatteryUI = -1;
            drawUI();
        }
        strlcpy(lastDisplayedDeviceId, currentDeviceId, sizeof(lastDisplayedDeviceId));
        lastUI = 0;
    }

    // 一定時間周期でCSVバッファを更新
    if (now - lastCSVMillis >= CSV_INTERVAL) {
        saveDataToCSV();
        lastCSVMillis = now;
    }

    // 一定時間周期でまとめて計測データをSDカードへ書き込む
    if (now - lastFlushMillis > CSV_FLUSH_INTERVAL) {
        flushCSVBuffer();
        lastFlushMillis = now;
    }

    // 一定時間周期でサーバへCSVを送信するトリガーを立てる
    if (now - lastSendMillis > SEND_INTERVAL) {
        sendFlag = true;
        lastSendMillis = now;
    }

    // 一定時間周期で画面更新（スリープ中は描画スキップ）
    if (now - lastUI > UI_UPDATE_MS) { 
        if (!isScreenSleeping) {
            drawUI(); // 必要な部分だけ数値の表示を更新する
        }
        lastUI = now;
    }

    delay(30);
}