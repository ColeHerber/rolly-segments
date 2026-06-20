#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include "secrets.h"
#include "pins.h"
#include "imu.h"
#include "can_bus.h"
#include "pwm_out.h"
#include "analog_in.h"
#include "web_ui.h"

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

constexpr uint32_t SSE_INTERVAL_MS       = 50;    // 20 Hz SSE push
constexpr uint32_t SERIAL_DBG_INTERVAL_MS = 500;  // 2 Hz serial JSON dump
constexpr uint32_t HEAP_WARN_INTERVAL_MS  = 10000;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
constexpr uint32_t HEAP_WARN_THRESHOLD    = 50000; // bytes

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static ImuData  imuData       = {};
static bool     imuOk         = false;
static bool     wifiDisabled  = false;
static bool     otaReady      = false;
static uint32_t lastWifiMs    = 0;
static uint32_t lastSseMs     = 0;
static uint32_t lastSerialMs  = 0;
static uint32_t lastHeapWarn  = 0;

// Pending CAN frames for the next SSE push (up to 8 buffered between pushes)
static CanFrame pendingCan[8];
static uint8_t  pendingCanCount = 0;

// ---------------------------------------------------------------------------
// LED heartbeat
// ---------------------------------------------------------------------------

enum class LedMode { SOLID_ON, BLINK_FAST, BLINK_SLOW };
static LedMode  ledMode     = LedMode::BLINK_SLOW;
static uint32_t lastLedMs   = 0;
static bool     ledState    = false;

static void updateLed() {
    if (ledMode == LedMode::SOLID_ON) {
        digitalWrite(LED_RED, HIGH);
        return;
    }
    const uint32_t interval = (ledMode == LedMode::BLINK_FAST) ? 100 : 1000;
    const uint32_t now = millis();
    if (now - lastLedMs >= interval) {
        ledState = !ledState;
        digitalWrite(LED_RED, ledState ? HIGH : LOW);
        lastLedMs = now;
    }
}

// ---------------------------------------------------------------------------
// WiFi / OTA (kept from original)
// ---------------------------------------------------------------------------

static void beginWifi() {
    if (strlen(WIFI_SSID) == 0 || strcmp(WIFI_SSID, "your-wifi-ssid") == 0) {
        wifiDisabled = true;
        dbgLog("[WiFi] Disabled — set WIFI_SSID in src/secrets.h");
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiMs = millis();
    dbgLog("[WiFi] Connecting to %s ...", WIFI_SSID);
}

static void beginOta() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        dbgLog("[OTA] Update started");
        digitalWrite(LED_RED, HIGH);
    });
    ArduinoOTA.onEnd([]()  { dbgLog("[OTA] Update finished"); });
    ArduinoOTA.onError([](ota_error_t err) { dbgLog("[OTA] Error %u", err); });
    ArduinoOTA.begin();
    otaReady = true;
    dbgLog("[OTA] Ready at %s (%s)", WiFi.localIP().toString().c_str(), OTA_HOSTNAME);
}

static void handleWifiAndOta() {
    if (wifiDisabled) return;

    if (WiFi.status() == WL_CONNECTED) {
        if (!otaReady) beginOta();
        ArduinoOTA.handle();
        return;
    }

    otaReady = false;
    const uint32_t now = millis();
    if (now - lastWifiMs >= WIFI_RETRY_INTERVAL_MS) {
        dbgLog("[WiFi] Retrying...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastWifiMs = now;
    }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup() {
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, HIGH);  // on during init

    Serial.begin(115200);
    Serial0.begin(115200);
    delay(400);

    dbgLog("=== ESP32-S3 CAN+IMU board boot ===");
    dbgLog("Chip: %s rev%d cores=%d %dMHz",
        ESP.getChipModel(), ESP.getChipRevision(),
        ESP.getChipCores(), ESP.getCpuFreqMHz());

    // IMU
    imuOk = imuInit();
    if (!imuOk) {
        ledMode = LedMode::SOLID_ON;  // solid = init error
        dbgLog("[IMU] INIT FAILED — check SPI wiring and WHO_AM_I above");
    } else {
        ledMode = LedMode::BLINK_FAST;  // fast = running, no WiFi yet
    }

    // CAN
    canInit(1000000);

    // PWM
    pwmInit();

    // ADC
    analogInInit();

    // WiFi
    beginWifi();

    // Web server (starts even without WiFi — browser can't reach it yet)
    webUiInit();

    // Pass WHO_AM_I to web UI for display
    extern void webUiSetImuWhoAmI(uint8_t);
    webUiSetImuWhoAmI(imuWhoAmI());

    digitalWrite(LED_RED, LOW);
    dbgLog("=== Setup complete. Free heap: %lu bytes ===", ESP.getFreeHeap());
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void loop() {
    handleWifiAndOta();

    // Update LED mode based on connection state
    if (imuOk) {
        ledMode = (WiFi.status() == WL_CONNECTED)
                    ? LedMode::BLINK_SLOW   // nominal — slow blink
                    : LedMode::BLINK_FAST;  // no wifi — fast blink
    }
    updateLed();

    // Read IMU when data-ready (INT1 high) or poll every loop
    if (imuAvailable()) {
        imuRead(imuData);
    }

    // Drain incoming CAN frames into pending buffer
    CanFrame f;
    while (canReceive(f) && pendingCanCount < 8) {
        pendingCan[pendingCanCount++] = f;
    }

    const uint32_t now = millis();

    // Push SSE at 20 Hz
    if (now - lastSseMs >= SSE_INTERVAL_MS) {
        lastSseMs = now;

        WebUiState state;
        state.imu     = imuData;
        state.imuOk   = imuOk;
        state.adc[0]  = analogInReadV(0);
        state.adc[1]  = analogInReadV(1);
        state.adc[2]  = analogInReadV(2);
        state.pwm[0]  = pwmGetUs(0);
        state.pwm[1]  = pwmGetUs(1);
        state.pwm[2]  = pwmGetUs(2);
        state.rssi    = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
        state.freeHeap = ESP.getFreeHeap();
        canGetStats(state.canTxOk, state.canTxErr, state.canRxOk, state.canRxErr);

        webUiPushSse(state, pendingCan, pendingCanCount);
        pendingCanCount = 0;
    }

    // Serial JSON debug dump at 2 Hz
    if (now - lastSerialMs >= SERIAL_DBG_INTERVAL_MS) {
        lastSerialMs = now;
        Serial.printf("{\"t\":%lu,\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
                       "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
                       "\"adc0\":%.3f,\"adc1\":%.3f,\"adc2\":%.3f,"
                       "\"pwm0\":%u,\"pwm1\":%u,\"pwm2\":%u,"
                       "\"heap\":%lu,\"rssi\":%d}\n",
            now,
            imuData.ax, imuData.ay, imuData.az,
            imuData.gx, imuData.gy, imuData.gz,
            analogInReadV(0), analogInReadV(1), analogInReadV(2),
            pwmGetUs(0), pwmGetUs(1), pwmGetUs(2),
            ESP.getFreeHeap(),
            (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0);
    }

    // Heap warning
    if (now - lastHeapWarn >= HEAP_WARN_INTERVAL_MS) {
        lastHeapWarn = now;
        const uint32_t heap = ESP.getFreeHeap();
        if (heap < HEAP_WARN_THRESHOLD) {
            dbgLog("[WARN] Low heap: %lu bytes", heap);
        }
    }
}
