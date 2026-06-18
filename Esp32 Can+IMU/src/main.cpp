#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include "secrets.h"

constexpr uint8_t RED_LED_PIN = 15;
constexpr uint32_t BLINK_INTERVAL_MS = 500;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;

bool ledOn = false;
bool otaReady = false;
bool wifiDisabled = false;
uint32_t lastBlinkMs = 0;
uint32_t lastWifiAttemptMs = 0;

void beginWifi() {
    if (strlen(WIFI_SSID) == 0 || strcmp(WIFI_SSID, "your-wifi-ssid") == 0) {
        wifiDisabled = true;
        Serial.println("WiFi/OTA disabled: set WIFI_SSID in src/secrets.h");
        Serial0.println("WiFi/OTA disabled: set WIFI_SSID in src/secrets.h");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttemptMs = millis();

    Serial.print("Connecting to WiFi for OTA: ");
    Serial.println(WIFI_SSID);
    Serial0.print("Connecting to WiFi for OTA: ");
    Serial0.println(WIFI_SSID);
}

void beginOta() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    if (strlen(OTA_PASSWORD) > 0) {
        ArduinoOTA.setPassword(OTA_PASSWORD);
    }

    ArduinoOTA.onStart([]() {
        Serial.println("OTA update started");
        Serial0.println("OTA update started");
        digitalWrite(RED_LED_PIN, HIGH);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("OTA update finished");
        Serial0.println("OTA update finished");
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA error %u\n", error);
        Serial0.printf("OTA error %u\n", error);
    });

    ArduinoOTA.begin();
    otaReady = true;

    Serial.print("OTA ready at ");
    Serial.print(WiFi.localIP());
    Serial.print(" as ");
    Serial.println(OTA_HOSTNAME);
    Serial0.print("OTA ready at ");
    Serial0.print(WiFi.localIP());
    Serial0.print(" as ");
    Serial0.println(OTA_HOSTNAME);
}

void handleWifiAndOta() {
    if (wifiDisabled) {
        return;
    }

    if (WiFi.status() == WL_CONNECTED) {
        if (!otaReady) {
            beginOta();
        }
        ArduinoOTA.handle();
        return;
    }

    otaReady = false;

    const uint32_t now = millis();
    if (now - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
        Serial.println("Retrying WiFi for OTA");
        Serial0.println("Retrying WiFi for OTA");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastWifiAttemptMs = now;
    }
}

void updateBlink() {
    const uint32_t now = millis();
    if (now - lastBlinkMs < BLINK_INTERVAL_MS) {
        return;
    }

    ledOn = !ledOn;
    digitalWrite(RED_LED_PIN, ledOn ? HIGH : LOW);
    lastBlinkMs = now;
}

void setup() {
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, LOW);

    Serial.begin(115200);
    Serial0.begin(115200);
    delay(500);
    Serial.println("ESP32 CAN+IMU red LED blink test with OTA");
    Serial0.println("ESP32 CAN+IMU red LED blink test with OTA");

    beginWifi();
}

void loop() {
    handleWifiAndOta();
    updateBlink();
}
