#pragma once
#include <stdint.h>

struct ImuData {
    float    ax, ay, az;   // g
    float    gx, gy, gz;   // degrees/s
    uint32_t ts;            // millis() at read time
};

// Returns true on successful init (WHO_AM_I match + config write).
// Always call imuWhoAmI() and check Serial output if this returns false —
// the actual WHO_AM_I byte is printed regardless.
bool        imuInit();

// True if INT1 (data-ready) pin is high.
bool        imuAvailable();

// Read one sample. Returns false if SPI fails.
bool        imuRead(ImuData& out);

// Human-readable status: "OK", "NOT_FOUND", or "SPI_ERROR"
const char* imuStatus();

// Raw WHO_AM_I byte read from register 0x0F (for debugging)
uint8_t     imuWhoAmI();
