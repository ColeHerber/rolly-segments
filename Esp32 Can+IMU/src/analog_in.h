#pragma once
#include <stdint.h>

// 3-channel ADC on IO1/IO2/IO3 (ADC1 — safe while WiFi is active).
// Hardware: 10k/20k voltage divider scales 0–5V → 0–3.33V.
// NOTE: R_BOT (20k) is not yet populated in Rev B schematic.
//       Until rework, readings directly reflect 0–3.3V at the pin.

void     analogInInit();

// Returns estimated voltage at header pin in volts.
// With divider populated: 0.0–5.0 V. Without: 0.0–3.3 V.
float    analogInReadV(uint8_t ch);

// Raw 12-bit ADC count (averaged over 16 samples).
uint16_t analogInReadRaw(uint8_t ch);
