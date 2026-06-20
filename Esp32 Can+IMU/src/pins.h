#pragma once

// IMU - ISM330BX on SPI2 (FSPI)
#define IMU_MOSI   35
#define IMU_MISO   37
#define IMU_SCK    36
#define IMU_CS     21
#define IMU_INT1   38   // data-ready interrupt
#define IMU_INT2   39   // FIFO watermark interrupt

// CAN — SN65HVD230 via ESP32 TWAI peripheral
#define CAN_TX     16
#define CAN_RX     17

// PWM outputs — 3.3V logic, servo/ESC capable
#define PWM_PIN_0   4   // CH1 — J7
#define PWM_PIN_1   5   // CH2 — J8
#define PWM_PIN_2   6   // CH3 — J9

// Analog inputs — 0–5V via 10k/20k voltage divider (R_BOT missing until rework)
#define ADC_PIN_0   1   // CH1 — J4
#define ADC_PIN_1   2   // CH2 — J5
#define ADC_PIN_2   3   // CH3 — J6

// Status LED (red, active high)
#define LED_RED    15
