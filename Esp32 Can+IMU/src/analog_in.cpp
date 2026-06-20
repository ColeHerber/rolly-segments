#include "analog_in.h"
#include "pins.h"
#include <Arduino.h>

static const uint8_t PINS[3] = { ADC_PIN_0, ADC_PIN_1, ADC_PIN_2 };

// Divider ratio: Vsig × R_BOT/(R_TOP+R_BOT) = Vsig × 20k/30k = 0.667
// With R_BOT unpopulated (Rev B), ratio = 1.0 (no division — reads 0–3.3V at pin)
// Set to 1.0 until R_BOT is soldered; change to 1.5 after rework.
static constexpr float DIVIDER_RATIO = 1.0f;  // TODO: change to 1.5f after R_BOT rework
static constexpr float VREF = 3.3f;
static constexpr float ADC_FULL_SCALE = 4095.0f;

void analogInInit() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // 0–3.3V input range
    for (uint8_t i = 0; i < 3; i++) {
        pinMode(PINS[i], INPUT);
    }
    Serial.println("[ADC] Init OK — 3 channels (12-bit, 16-sample average)");
    Serial0.println("[ADC] Init OK");
}

uint16_t analogInReadRaw(uint8_t ch) {
    if (ch >= 3) return 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 16; i++) sum += analogRead(PINS[ch]);
    return (uint16_t)(sum >> 4);  // divide by 16
}

float analogInReadV(uint8_t ch) {
    const uint16_t raw = analogInReadRaw(ch);
    return (raw / ADC_FULL_SCALE) * VREF * (1.0f / DIVIDER_RATIO);
}
