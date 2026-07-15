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

#define DEVICE_ID "YUUKI" // 児童の名前（デバイスごとに変える）

BLEScan* pBLEScan;
BLEAdvertising* pAdvertising;
TaskHandle_t bleTaskHandle;

portMUX_TYPE sharedStateMux = portMUX_INITIALIZER_UNLOCKED;

// =========================
// 歩数
// =========================
volatile int stepCount = 0;

// =========================
// BLE（相対距離）
// =========================
volatile int latestRSSI = -100;
volatile float distanceMeter = -1;

// =========================
// 複数ノード管理
// =========================
struct DeviceInfo {
    String id;
    int rssi;
    float distance;
    unsigned long lastSeen;
};

DeviceInfo devices[20];
int deviceCount = 0;

// =========================
// SD / CSV
// =========================
char csvFileName[32] = "";
unsigned long lastCSVMillis = 0;
const unsigned long CSV_INTERVAL = 10000;

// =========================
// SDカードの初期化
// =========================
void initSDCard() {
    if (!SD.begin(4)) {
        Serial.println("SD card initialization failed");
        return;
    }

    Serial.println("SD card initialized");
}

void createNewCSVFile() {
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);

    sprintf(csvFileName,
            "/data_%04d%02d%02d_%02d%02d%02d.csv",
            timeinfo->tm_year + 1900,
            timeinfo->tm_mon + 1,
            timeinfo->tm_mday,
            timeinfo->tm_hour,
            timeinfo->tm_min,
            timeinfo->tm_sec
        );

    File file = SD.open(csvFileName, FILE_WRITE);

    if (file) {
        file.println("Timestamp,Steps,Distance(m)");
        file.close();

        Serial.printf(
            "New CSV file created: %s\n",
            csvFileName
        );
    }
}

// =========================
// CSVへのデータの書き込み
// =========================
void saveDataToCSV(unsigned long timestamp, int steps, float distance) {
    if (csvFileName[0] == '\0')
        return;

    File file = SD.open(csvFileName, FILE_APPEND);

    if (file) {
        file.printf("%lu,%d,%.2f\n", timestamp, steps, distance);
        file.close();
    }
}

// =========================
// 歩数計算
// =========================
void updateStepCount() {
    static float gravity = 1.0f;
    static float filtered = 0.0f;
    static float prevFiltered = 0.0f;
    static bool rising = false;
    static unsigned long lastStepMillis = 0;
    static bool calibrated = false;

    const float ALPHA = 0.92f;
    const float STEP_THRESHOLD = 0.18f;
    const unsigned long STEP_INTERVAL = 300;

    if (!calibrated) {
        for (int i = 0; i < 40; i++) {
            float ax, ay, az;

            M5.Imu.getAccelData(&ax, &ay, &az);

            float accelMagnitude =
                sqrt(ax * ax + ay * ay + az * az);

            gravity = gravity * 0.9f + accelMagnitude * 0.1f;

            delay(20);
        }

        calibrated = true;
    }

    float ax, ay, az;

    M5.Imu.getAccelData(&ax, &ay, &az);

    float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);

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

// =========================
// 距離計算
// =========================
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

// =========================
// ノード更新
// =========================
void updateDevice(String id, int rssi) {
    float distance = calculateDistance(rssi);

    for (int i = 0; i < deviceCount; i++) {
        if (devices[i].id == id) {

            devices[i].rssi = rssi;
            devices[i].distance = distance;
            devices[i].lastSeen = millis();

            return;
        }
    }

    if (deviceCount < 20) {
        devices[deviceCount].id = id;
        devices[deviceCount].rssi = rssi;
        devices[deviceCount].distance = distance;
        devices[deviceCount].lastSeen = millis();

        deviceCount++;
    }
}

// =========================
// BLE callback
// =========================
class MyCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) {
        String name = device.getName().c_str();

        if (!name.startsWith("NODE") || name == DEVICE_ID)
            return;

        int rssi = device.getRSSI();
        float distance = calculateDistance(rssi);

        updateDevice(name, rssi);

        portENTER_CRITICAL(&sharedStateMux);

        latestRSSI = rssi;
        distanceMeter = distance;

        portEXIT_CRITICAL(&sharedStateMux);

        Serial.printf("%s RSSI=%d Dist=%.2f\n", name.c_str(), rssi, distance);
    }
};

// =========================
// BLE task
// =========================
void bleTask(void *arg) {
    while (true) {
        pBLEScan->start(1, false);
        pBLEScan->clearResults();

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// =========================
// child_idの取得
// =========================
int getChildId(const String& deviceName) {
    // DBから児童の名前に対応したidを照合する

    return 1;
}

// =========================
// UI描画
// =========================
void drawUI() {
    M5.Display.fillScreen(TFT_NAVY);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(5);
    M5.Display.setCursor(20, 20);
    M5.Display.println(DEVICE_ID);
    M5.Display.setTextSize(3);
    M5.Display.setCursor(20, 90);

    M5.Display.printf("STEP: %d", stepCount);

    int y = 140;

    M5.Display.setTextSize(2);

    for (int i = 0; i < deviceCount; i++) {
        if (millis() - devices[i].lastSeen > 10000)
            continue;

        M5.Display.setCursor(20, y);
        M5.Display.printf("%s %.1fm", devices[i].id.c_str(), devices[i].distance);

        y += 25;
    }

    int battery = M5.Power.getBatteryLevel();

    M5.Display.setTextColor(TFT_YELLOW);
    M5.Display.setCursor(M5.Display.width() - 110, M5.Display.height() - 30);

    M5.Display.printf("%d%%", battery);
}

// =========================
// デバイスのセットアップ
// =========================
void setup() {
    auto cfg = M5.config();

    cfg.serial_baudrate = 115200;
    cfg.clear_display = true;

    M5.begin(cfg);

    Serial.begin(115200);

    delay(100);

    M5.Display.setRotation(1);

    initSDCard();
    createNewCSVFile();

    // BLE
    BLEDevice::init(DEVICE_ID);

    // Advertising
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

    // Wi-Fiへの接続を行う
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while( WiFi.status() != WL_CONNECTED) {
        delay(500); 
        M5.Lcd.print("."); 
    }
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.println("WiFi connected");
    M5.Lcd.print("IP address = ");
    M5.Lcd.println(WiFi.localIP()); // デバイスのローカルIPアドレス

    // Wi-Fi接続後
    configTime(9 * 3600, 0, "pool.ntp.org", "ntp.jst.mfeed.ad.jp");

    struct tm timeinfo;

    // 同期完了まで待つ
    while (!getLocalTime(&timeinfo)) {
        Serial.println("NTP同期待ち...");
        delay(500);
    }

    Serial.println("NTP同期完了");
}

// =========================
// Wi-Fi経由のデータ送信
// =========================
void sendDataToServer(unsigned long timestamp, int steps, float distanceSnapshot) {
    // Wi-Fiのコネクションを確認
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected!");
        return;
    }

    // ISOタイムスタンプ生成
    time_t now = time(nullptr);
    struct tm* t = gmtime(&now);
    char isoTime[32];

    sprintf(
        isoTime, "%04d-%02d-%02dT%02d:%02d:%02d.000Z",
        t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
        t->tm_hour, t->tm_min, t->tm_sec
    );

    // JSON組み立て（512バイトに拡張）
    StaticJsonDocument<512> doc;

    // 自分の児童IDをDBから参照して設定
    doc["child_id"] = getChildId(DEVICE_ID);

    // 歩数情報を格納
    JsonObject singledata = doc.createNestedObject("singledata");
    singledata["date"] = isoTime; // タイムスタンプ
    singledata["steps"] = steps; // 歩数情報

    // 相対距離情報を格納（近くにいるデバイス分だけ追加）
    JsonArray distances = doc.createNestedArray("distances");
    for (int i = 0; i < deviceCount; i++) {
        // 10秒以上検出されていないデバイスは送信しない
        if (millis() - devices[i].lastSeen > 10000) 
            continue;

        JsonObject dist = distances.createNestedObject();

        dist["date"] = isoTime; // タイムスタンプ
        // dist["with_child"] = getChildId(devices[i].id); // 測定した相手のID
        dist["with_child"] = 2; // テスト用に定数でID=2を設定
        dist["distance"] = devices[i].distance; // 相対距離情報
    }

    // JSONを文字列へ変換
    String jsonStr;
    serializeJson(doc, jsonStr);
    Serial.println("Sending: " + jsonStr);

    // HTTPのPOSTでサーバへデータを送信するようにする
    HTTPClient http;
    http.begin(API_URL); // APIサーバのURLを設定
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonStr);

    if (httpResponseCode > 0) {
        Serial.println("HTTP Response: " + String(httpResponseCode));
        String response = http.getString();
        Serial.println("Response: " + response);
    } else {
        Serial.println("Error: " + String(httpResponseCode));
    }

    http.end();
}
    
// =========================
// メインループ
// =========================
void loop() {
    updateStepCount(); // 歩数計算

    unsigned long lastUI = 0;
    unsigned long now = millis();

    if (now - lastCSVMillis > CSV_INTERVAL) {
        float distanceSnapshot;

        portENTER_CRITICAL(&sharedStateMux);

        distanceSnapshot = distanceMeter;

        portEXIT_CRITICAL(&sharedStateMux);

        saveDataToCSV(now / 1000, stepCount, distanceSnapshot);
        sendDataToServer(now / 1000, stepCount, distanceSnapshot);

        lastCSVMillis = now;
    }

    if (millis() - lastUI > 5000) {
        drawUI();
        lastUI = millis();
    }

    delay(30);
}