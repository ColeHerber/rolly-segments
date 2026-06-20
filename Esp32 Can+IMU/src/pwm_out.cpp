#include "pwm_out.h"
#include "pins.h"
#include <Arduino.h>

static const uint8_t PINS[3]  = { PWM_PIN_0, PWM_PIN_1, PWM_PIN_2 };
static uint16_t      currentUs[3] = { 1500, 1500, 1500 };
static float         freqHz[3]    = { 50.0f, 50.0f, 50.0f };

static constexpr uint8_t RESOLUTION_BITS = 16;
// LEDC channels 0-2 assigned to PWM outputs 0-2
static const uint8_t LEDC_CH[3] = { 0, 1, 2 };

// Convert microseconds to LEDC duty count for the channel's current frequency.
static uint32_t usToDuty(uint8_t ch, uint16_t us) {
    const float periodUs = 1e6f / freqHz[ch];
    const uint32_t maxDuty = (1u << RESOLUTION_BITS) - 1;
    return (uint32_t)((us / periodUs) * maxDuty);
}

void pwmInit() {
    for (uint8_t i = 0; i < 3; i++) {
        ledcSetup(LEDC_CH[i], (uint32_t)freqHz[i], RESOLUTION_BITS);
        ledcAttachPin(PINS[i], LEDC_CH[i]);
        ledcWrite(LEDC_CH[i], usToDuty(i, currentUs[i]));
    }
    Serial.println("[PWM] Init OK — 3 channels at 50 Hz, 1500 µs");
    Serial0.println("[PWM] Init OK");
}

void pwmSetUs(uint8_t ch, uint16_t us) {
    if (ch >= 3) return;
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;
    currentUs[ch] = us;
    ledcWrite(LEDC_CH[ch], usToDuty(ch, us));
}

void pwmSetFreqHz(uint8_t ch, float hz) {
    if (ch >= 3 || hz < 1.0f || hz > 400.0f) return;
    freqHz[ch] = hz;
    ledcSetup(LEDC_CH[ch], (uint32_t)hz, RESOLUTION_BITS);
    ledcAttachPin(PINS[ch], LEDC_CH[ch]);
    ledcWrite(LEDC_CH[ch], usToDuty(ch, currentUs[ch]));
}

uint16_t pwmGetUs(uint8_t ch) {
    return (ch < 3) ? currentUs[ch] : 0;
}
