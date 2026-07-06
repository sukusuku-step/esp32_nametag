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

#define DEVICE_ID "NODE01"

BLEScan* pBLEScan;
BLEAdvertising* pAdvertising;
TaskHandle_t bleTaskHandle;

portMUX_TYPE sharedStateMux = portMUX_INITIALIZER_UNLOCKED;

// =========================
// 歩数
// =========================
volatile int stepCount = 0;

// =========================
// BLE
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
// SD
// =========================
void initSDCard()
{
    if (!SD.begin(4)) {
        Serial.println("SD card initialization failed");
        return;
    }

    Serial.println("SD card initialized");
}

void createNewCSVFile()
{
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);

    sprintf(csvFileName,
            "/data_%04d%02d%02d_%02d%02d%02d.csv",
            timeinfo->tm_year + 1900,
            timeinfo->tm_mon + 1,
            timeinfo->tm_mday,
            timeinfo->tm_hour,
            timeinfo->tm_min,
            timeinfo->tm_sec);

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

void saveDataToCSV(
    unsigned long timestamp,
    int steps,
    float distance)
{
    if (csvFileName[0] == '\0')
        return;

    File file =
        SD.open(csvFileName, FILE_APPEND);

    if (file) {

        file.printf(
            "%lu,%d,%.2f\n",
            timestamp,
            steps,
            distance
        );

        file.close();
    }
}

// =========================
// 距離計算
// =========================
float calculateDistance(int rssi)
{
    int txPower = -59;

    if (rssi == 0)
        return -1;

    float ratio =
        rssi * 1.0 / txPower;

    if (ratio < 1.0) {

        return pow(ratio, 10);
    }

    return
        0.89976 *
        pow(ratio, 7.7095)
        + 0.111;
}

// =========================
// ノード更新
// =========================
void updateDevice(
    String id,
    int rssi)
{
    float distance =
        calculateDistance(rssi);

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
class MyCallbacks :
    public BLEAdvertisedDeviceCallbacks {

    void onResult(
        BLEAdvertisedDevice device)
    {
        String name =
            device.getName().c_str();

        if (!name.startsWith("NODE"))
            return;

        if (name == DEVICE_ID)
            return;

        int rssi =
            device.getRSSI();

        float distance =
            calculateDistance(rssi);

        updateDevice(name, rssi);

        portENTER_CRITICAL(
            &sharedStateMux);

        latestRSSI = rssi;
        distanceMeter = distance;

        portEXIT_CRITICAL(
            &sharedStateMux);

        Serial.printf(
            "%s RSSI=%d Dist=%.2f\n",
            name.c_str(),
            rssi,
            distance
        );
    }
};

// =========================
// BLE task
// =========================
void bleTask(void *arg)
{
    while (true) {

        pBLEScan->start(
            1,
            false
        );

        pBLEScan->clearResults();

        vTaskDelay(
            200 / portTICK_PERIOD_MS
        );
    }
}

// =========================
// UI
// =========================
void drawUI()
{
    M5.Display.fillScreen(
        TFT_NAVY);

    M5.Display.setTextColor(
        WHITE);

    M5.Display.setTextSize(5);

    M5.Display.setCursor(20, 20);
    M5.Display.println(DEVICE_ID);

    M5.Display.setTextSize(3);

    M5.Display.setCursor(20, 90);

    M5.Display.printf(
        "STEP: %d",
        stepCount
    );

    int y = 140;

    M5.Display.setTextSize(2);

    for (int i = 0; i < deviceCount; i++) {

        if (
            millis()
            - devices[i].lastSeen
            > 10000)
            continue;

        M5.Display.setCursor(
            20,
            y
        );

        M5.Display.printf(
            "%s %.1fm",
            devices[i].id.c_str(),
            devices[i].distance
        );

        y += 25;
    }

    int battery =
        M5.Power.getBatteryLevel();

    M5.Display.setTextColor(
        TFT_YELLOW);

    M5.Display.setCursor(
        M5.Display.width() - 110,
        M5.Display.height() - 30
    );

    M5.Display.printf(
        "%d%%",
        battery
    );
}

// =========================
// setup
// =========================
void setup()
{
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
    BLEDevice::init(
        DEVICE_ID
    );

    // Advertising
    pAdvertising =
        BLEDevice::getAdvertising();

    BLEAdvertisementData advData;

    advData.setName(
        DEVICE_ID
    );

    pAdvertising->setAdvertisementData(
        advData
    );

    pAdvertising->start();

    // Scan
    pBLEScan =
        BLEDevice::getScan();

    pBLEScan->setAdvertisedDeviceCallbacks(
        new MyCallbacks()
    );

    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(80);

    xTaskCreatePinnedToCore(
        bleTask,
        "BLE",
        4096,
        NULL,
        2,
        &bleTaskHandle,
        1
    );

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while( WiFi.status() != WL_CONNECTED) {
        delay(500); 
        M5.Lcd.print("."); 
    }
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.println("WiFi connected");
    M5.Lcd.print("IP address = ");
    M5.Lcd.println(WiFi.localIP());
}

//HTTPリクエスト
void sendDataToServer(int  time , int stepCount, float distanceSnapshot) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(API_URL);
    http.addHeader("Content-Type", "application/json");

    // JSON 作成
    StaticJsonDocument<200> doc;
    doc["time"] = time;
    doc["deviceId"]  = stepCount;
    doc["distance"] = distanceSnapshot;

    String jsonStr;
    serializeJson(doc, jsonStr);

    int httpResponseCode = http.POST(jsonStr);

    if (httpResponseCode > 0) {
      String response = http.getString();
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.println("HTTP Response code: " + String(httpResponseCode));
      M5.Lcd.println("Response: " + response);

      if (httpResponseCode == 200) {
        M5.Lcd.println("Register Confirmed");
        delay(1000);
      }
 
    } else {
      M5.Lcd.println("Error on sending POST: " + String(httpResponseCode));
    }

    http.end();
  } else {
    M5.Lcd.println("WiFi not connected!");
  }
}

// =========================
// loop
// =========================
void loop()
{
    static float gravity = 1.0f;
    static float filtered = 0.0f;
    static float prevFiltered = 0.0f;

    static bool rising = false;

    static unsigned long lastStepMillis = 0;

    static unsigned long lastUI = 0;

    static bool calibrated = false;

    const float ALPHA = 0.92f;
    const float STEP_THRESHOLD = 0.18f;
    const unsigned long STEP_INTERVAL = 300;

    if (!calibrated) {

        for (int i = 0; i < 40; i++) {

            float ax, ay, az;

            M5.Imu.getAccelData(
                &ax,
                &ay,
                &az
            );

            float accelMagnitude =
                sqrt(
                    ax * ax +
                    ay * ay +
                    az * az
                );

            gravity =
                gravity * 0.9f +
                accelMagnitude * 0.1f;

            delay(20);
        }

        calibrated = true;
    }

    float ax, ay, az;

    M5.Imu.getAccelData(
        &ax,
        &ay,
        &az
    );

    float accelMagnitude =
        sqrt(
            ax * ax +
            ay * ay +
            az * az
        );

    gravity =
        gravity * ALPHA +
        accelMagnitude * (1.0f - ALPHA);

    filtered =
        accelMagnitude - gravity;

    bool currentRising =
        filtered > prevFiltered;

    if (
        rising &&
        !currentRising &&
        prevFiltered > STEP_THRESHOLD
    ) {

        unsigned long now =
            millis();

        if (
            now - lastStepMillis >
            STEP_INTERVAL
        ) {

            stepCount++;

            lastStepMillis = now;

            Serial.printf(
                "STEP %d\n",
                stepCount
            );
        }
    }

    rising = currentRising;
    prevFiltered = filtered;

    unsigned long now =
        millis();

    if (
        now - lastCSVMillis >
        CSV_INTERVAL
    ) {

        float distanceSnapshot;

        portENTER_CRITICAL(
            &sharedStateMux);

        distanceSnapshot =
            distanceMeter;

        portEXIT_CRITICAL(
            &sharedStateMux);

        saveDataToCSV(
            now / 1000,
            stepCount,
            distanceSnapshot
        );

        sendDataToServer(
            now / 1000,
            stepCount,
            distanceSnapshot
        );

        lastCSVMillis = now;
    }

    if (
        millis() - lastUI >
        5000
    ) {

        drawUI();

        lastUI = millis();
    }

    delay(30);
}