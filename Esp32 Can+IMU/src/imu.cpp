#include "imu.h"
#include "pins.h"
#include "web_ui.h"
#include <Arduino.h>
#include <SPI.h>

// Register map - ISM330BX family.
namespace reg {
    constexpr uint8_t IF_CFG      = 0x03;  // interface config
    constexpr uint8_t INT1_CTRL   = 0x0D;  // interrupt routing
    constexpr uint8_t INT2_CTRL   = 0x0E;
    constexpr uint8_t WHO_AM_I    = 0x0F;
    constexpr uint8_t CTRL1       = 0x10;  // accel ODR / op-mode
    constexpr uint8_t CTRL2       = 0x11;  // gyro ODR / full-scale
    constexpr uint8_t CTRL3       = 0x12;  // BDU, IF_INC, SW_RESET
    constexpr uint8_t CTRL6       = 0x15;  // gyro full-scale
    constexpr uint8_t CTRL8       = 0x17;  // accel full-scale
    constexpr uint8_t STATUS_REG  = 0x1E;  // bit0=XLDA, bit1=GDA
    constexpr uint8_t OUTX_L_G    = 0x22;  // gyro XYZ (6 bytes)
    constexpr uint8_t OUTZ_L_A    = 0x28;  // accel ZYX (6 bytes)
}

constexpr uint8_t WHO_AM_I_EXPECTED = 0x71;

#ifndef IMU_SPI_DEBUG
#define IMU_SPI_DEBUG 0
#endif

#ifndef IMU_SPI_SCOPE_LOOP
#define IMU_SPI_SCOPE_LOOP 0
#endif

#ifndef IMU_SPI_SCOPE_LOOP_COUNT
#define IMU_SPI_SCOPE_LOOP_COUNT 300
#endif

#ifndef IMU_SPI_IFCFG_TEST
#define IMU_SPI_IFCFG_TEST 0
#endif

#ifndef IMU_SPI_CS_LOW_HOLD_MS
#define IMU_SPI_CS_LOW_HOLD_MS 0
#endif

// Accel sensitivity for ±8g: 0.244 mg/LSB
constexpr float ACCEL_SCALE = 0.000244f;
// Gyro sensitivity for ±2000 dps: 70 mdps/LSB
constexpr float GYRO_SCALE = 0.070f;
constexpr uint32_t IMU_SPI_HZ = 1000000;
constexpr uint8_t ISM330BX_I2C_ADDR_LOW = 0x6A;
constexpr uint8_t ISM330BX_I2C_ADDR_HIGH = 0x6B;
constexpr uint16_t BITBANG_I2C_DELAY_US = 20;
constexpr uint16_t BITBANG_SPI_EDGE_DELAY_US = 25;
constexpr uint8_t IF_CFG_I2C_I3C_DISABLE = 0x01;
constexpr uint8_t IF_CFG_SIM_3WIRE = 0x04;

static SPIClass imuSpi(FSPI);
static uint8_t  whoAmIByte = 0x00;
static bool     initOk = false;
static bool     useI2c = false;
static uint8_t  i2cAddr = ISM330BX_I2C_ADDR_LOW;
static uint8_t  spiMode = SPI_MODE3;

static void applyI2cAddressBias() {
    // SDO/TA0 selects the I2C address. Keep it biased while using I2C fallback.
    pinMode(IMU_MISO, i2cAddr == ISM330BX_I2C_ADDR_HIGH ? INPUT_PULLUP : INPUT_PULLDOWN);
}

static void i2cDelay() {
    delayMicroseconds(BITBANG_I2C_DELAY_US);
}

static void i2cRelease(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
}

static void i2cLow(uint8_t pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

static void i2cStart() {
    i2cRelease(IMU_MOSI);  // SDA
    i2cRelease(IMU_SCK);   // SCL
    i2cDelay();
    i2cLow(IMU_MOSI);
    i2cDelay();
    i2cLow(IMU_SCK);
    i2cDelay();
}

static void i2cStop() {
    i2cLow(IMU_MOSI);
    i2cDelay();
    i2cRelease(IMU_SCK);
    i2cDelay();
    i2cRelease(IMU_MOSI);
    i2cDelay();
}

static bool i2cWriteByte(uint8_t byte) {
    for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
        if (byte & mask) {
            i2cRelease(IMU_MOSI);
        } else {
            i2cLow(IMU_MOSI);
        }
        i2cDelay();
        i2cRelease(IMU_SCK);
        i2cDelay();
        i2cLow(IMU_SCK);
    }

    i2cRelease(IMU_MOSI);
    i2cDelay();
    i2cRelease(IMU_SCK);
    i2cDelay();
    const bool ack = digitalRead(IMU_MOSI) == LOW;
    i2cLow(IMU_SCK);
    i2cDelay();
    return ack;
}

static uint8_t i2cReadByte(bool ack) {
    uint8_t byte = 0;
    i2cRelease(IMU_MOSI);
    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        i2cRelease(IMU_SCK);
        i2cDelay();
        if (digitalRead(IMU_MOSI)) byte |= 1;
        i2cLow(IMU_SCK);
        i2cDelay();
    }

    if (ack) {
        i2cLow(IMU_MOSI);
    } else {
        i2cRelease(IMU_MOSI);
    }
    i2cDelay();
    i2cRelease(IMU_SCK);
    i2cDelay();
    i2cLow(IMU_SCK);
    i2cRelease(IMU_MOSI);
    return byte;
}

static bool i2cReadBurst(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len, bool biasAddressPin = true) {
    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);
    if (biasAddressPin) {
        applyI2cAddressBias();
    }

    i2cStart();
    if (!i2cWriteByte(addr << 1)) {
        i2cStop();
        return false;
    }
    if (!i2cWriteByte(reg)) {
        i2cStop();
        return false;
    }
    i2cStart();
    if (!i2cWriteByte((addr << 1) | 1)) {
        i2cStop();
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = i2cReadByte(i + 1 < len);
    }
    i2cStop();
    return true;
}

static bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t& value, bool biasAddressPin = true) {
    return i2cReadBurst(addr, reg, &value, 1, biasAddressPin);
}

static bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);
    applyI2cAddressBias();

    i2cStart();
    if (!i2cWriteByte(addr << 1)) {
        i2cStop();
        return false;
    }
    if (!i2cWriteByte(reg)) {
        i2cStop();
        return false;
    }
    if (!i2cWriteByte(value)) {
        i2cStop();
        return false;
    }
    i2cStop();
    return true;
}

static bool runI2cAliveProbe() {
    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);

    bool found = false;
    uint8_t foundAddr = ISM330BX_I2C_ADDR_LOW;
    for (uint8_t pass = 0; pass < 2; pass++) {
        if (pass == 0) {
            pinMode(IMU_MISO, INPUT_PULLDOWN); // bias SDO/TA0 low -> address 0x6A
            dbgLog("[IMU] I2C probe with SDO/TA0 pulldown");
        } else {
            pinMode(IMU_MISO, INPUT_PULLUP);   // bias SDO/TA0 high -> address 0x6B
            dbgLog("[IMU] I2C probe with SDO/TA0 pullup");
        }

        uint8_t whoLow = 0x00;
        uint8_t whoHigh = 0x00;
        const bool ackLow = i2cReadReg(ISM330BX_I2C_ADDR_LOW, reg::WHO_AM_I, whoLow);
        const bool ackHigh = i2cReadReg(ISM330BX_I2C_ADDR_HIGH, reg::WHO_AM_I, whoHigh);
        dbgLog("[IMU] I2C addr 0x6A ack=%d WHO=0x%02X | addr 0x6B ack=%d WHO=0x%02X",
               ackLow, whoLow, ackHigh, whoHigh);
        if (!found && ackLow && whoLow == WHO_AM_I_EXPECTED) {
            foundAddr = ISM330BX_I2C_ADDR_LOW;
            found = true;
        }
        if (!found && ackHigh && whoHigh == WHO_AM_I_EXPECTED) {
            foundAddr = ISM330BX_I2C_ADDR_HIGH;
            found = true;
        }
    }

    i2cAddr = foundAddr;
    i2cRelease(IMU_MOSI);
    i2cRelease(IMU_SCK);
    if (found) {
        applyI2cAddressBias();
    } else {
        pinMode(IMU_MISO, INPUT);
    }
    return found;
}

static uint8_t spiReadWithSettings(uint8_t reg, uint8_t mode, uint32_t hz) {
    digitalWrite(IMU_CS, HIGH);
    delayMicroseconds(5);
    imuSpi.beginTransaction(SPISettings(hz, MSBFIRST, mode));
    digitalWrite(IMU_CS, LOW);
    delayMicroseconds(2);
    imuSpi.transfer(reg | 0x80);  // bit7=1 → read
    const uint8_t val = imuSpi.transfer(0x00);
    digitalWrite(IMU_CS, HIGH);
    imuSpi.endTransaction();
    return val;
}

static uint8_t spiReadWithMode(uint8_t reg, uint8_t mode) {
    return spiReadWithSettings(reg, mode, IMU_SPI_HZ);
}

static uint8_t spiRead(uint8_t reg) {
    return spiReadWithMode(reg, spiMode);
}

static void spiWrite(uint8_t reg, uint8_t val) {
    imuSpi.beginTransaction(SPISettings(IMU_SPI_HZ, MSBFIRST, spiMode));
    digitalWrite(IMU_CS, LOW);
    imuSpi.transfer(reg & 0x7F);  // bit7=0 → write
    imuSpi.transfer(val);
    digitalWrite(IMU_CS, HIGH);
    imuSpi.endTransaction();
}

// Read `len` bytes starting at `reg` using auto-increment (IF_INC must be set).
static void spiBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    imuSpi.beginTransaction(SPISettings(IMU_SPI_HZ, MSBFIRST, spiMode));
    digitalWrite(IMU_CS, LOW);
    imuSpi.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buf[i] = imuSpi.transfer(0x00);
    digitalWrite(IMU_CS, HIGH);
    imuSpi.endTransaction();
}

#if IMU_SPI_DEBUG || IMU_SPI_SCOPE_LOOP
struct BitBangSpiResult {
    uint16_t raw;
    uint8_t  value;
};

static const char* spiModeLabel(uint8_t mode) {
    return mode == SPI_MODE0 ? "mode0" : "mode3";
}

static void logSpiGpioState(const char* label) {
    dbgLog("[IMU][SPIDBG] GPIO %s CS=%d SCK=%d MOSI=%d MISO=%d",
           label,
           digitalRead(IMU_CS),
           digitalRead(IMU_SCK),
           digitalRead(IMU_MOSI),
           digitalRead(IMU_MISO));
}

static void prepareBitBangSpi(uint8_t mode) {
    const uint8_t idleClock = (mode == SPI_MODE3) ? HIGH : LOW;

    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_SCK, idleClock);
    pinMode(IMU_SCK, OUTPUT);
    digitalWrite(IMU_MOSI, LOW);
    pinMode(IMU_MOSI, OUTPUT);
    pinMode(IMU_MISO, INPUT);
    delayMicroseconds(100);
}

static uint8_t clockBitBangSpiBit(uint8_t mode, bool txBit) {
    uint8_t rxBit = 0;
    digitalWrite(IMU_MOSI, txBit ? HIGH : LOW);
    delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);

    if (mode == SPI_MODE3) {
        digitalWrite(IMU_SCK, LOW);
        delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);
        digitalWrite(IMU_SCK, HIGH);
        delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);
        rxBit = digitalRead(IMU_MISO) ? 1 : 0;
    } else {
        digitalWrite(IMU_SCK, HIGH);
        delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);
        rxBit = digitalRead(IMU_MISO) ? 1 : 0;
        digitalWrite(IMU_SCK, LOW);
        delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);
    }

    return rxBit;
}

static BitBangSpiResult bitBangSpiReadReg(uint8_t reg, uint8_t mode) {
    prepareBitBangSpi(mode);

    const uint16_t tx = static_cast<uint16_t>(reg | 0x80) << 8;
    uint16_t raw = 0;

    digitalWrite(IMU_CS, LOW);
    delayMicroseconds(10);
    for (uint8_t bit = 0; bit < 16; bit++) {
        const bool txBit = (tx & (0x8000 >> bit)) != 0;
        raw = static_cast<uint16_t>((raw << 1) | clockBitBangSpiBit(mode, txBit));
    }
    delayMicroseconds(5);
    digitalWrite(IMU_CS, HIGH);

    const uint8_t idleClock = (mode == SPI_MODE3) ? HIGH : LOW;
    digitalWrite(IMU_SCK, idleClock);
    digitalWrite(IMU_MOSI, LOW);

    BitBangSpiResult result;
    result.raw = raw;
    result.value = raw & 0xFF;
    return result;
}

static void runHardwareSpiSweep(const char* label) {
    constexpr uint32_t speeds[] = {100000, 400000, 1000000};
    constexpr uint8_t modes[] = {SPI_MODE3, SPI_MODE0};

    dbgLog("[IMU][SPIDBG] HW SPI sweep: %s", label);
    for (uint8_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); i++) {
        for (uint8_t j = 0; j < sizeof(modes) / sizeof(modes[0]); j++) {
            const uint8_t who = spiReadWithSettings(reg::WHO_AM_I, modes[j], speeds[i]);
            dbgLog("[IMU][SPIDBG] HW %lu Hz %s WHO=0x%02X",
                   static_cast<unsigned long>(speeds[i]),
                   spiModeLabel(modes[j]),
                   who);
        }
    }
}

static void logSpiDebugHeader() {
    dbgLog("[IMU][SPIDBG] ==================================================");
    dbgLog("[IMU][SPIDBG] ISM330BX SPI debug sequence");
    dbgLog("[IMU][SPIDBG] Order: I2C alive -> HW SPI -> bit-bang SPI -> SDO/TA0 strap -> CS gate");
    dbgLog("[IMU][SPIDBG] If IF_CFG was used earlier, cold power-cycle before trusting I2C tests");
    dbgLog("[IMU][SPIDBG] Expected SPI WHO: CS low, MOSI=0x8F, 16 clocks, MISO byte2=0x71");
    dbgLog("[IMU][SPIDBG] ==================================================");
}

static void logSpiDebugPowerCyclePrompt() {
    dbgLog("[IMU][SPIDBG] Debug sequence complete.");
    dbgLog("[IMU][SPIDBG] Before rerunning after any interface-mode experiment: physically power-cycle the board.");
}

static void runBitBangSpiSweep(const char* label) {
    constexpr uint8_t modes[] = {SPI_MODE3, SPI_MODE0};

    dbgLog("[IMU][SPIDBG] bit-banged SPI sweep: %s", label);
    for (uint8_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        const BitBangSpiResult who = bitBangSpiReadReg(reg::WHO_AM_I, modes[i]);
        dbgLog("[IMU][SPIDBG] BB %s raw=0x%04X WHO=0x%02X",
               spiModeLabel(modes[i]), who.raw, who.value);
    }
}

static bool readWhoNoAddressBias(uint8_t addr, uint8_t& who) {
    return i2cReadReg(addr, reg::WHO_AM_I, who, false);
}

static void runSdoStrapDriveTest() {
    dbgLog("[IMU][SPIDBG] SDO/TA0 drive test: actively drive IO37 low/high and watch I2C address");
    dbgLog("[IMU][SPIDBG] expected if IO37 reaches SDO/TA0: LOW -> 0x6A, HIGH -> 0x6B");

    struct PassResult {
        bool ackLowAddr;
        bool ackHighAddr;
        uint8_t whoLowAddr;
        uint8_t whoHighAddr;
    };

    auto runPass = [](uint8_t level) {
        digitalWrite(IMU_CS, HIGH);
        pinMode(IMU_CS, OUTPUT);
        digitalWrite(IMU_MISO, level);
        pinMode(IMU_MISO, OUTPUT);
        delay(3);

        PassResult result;
        result.whoLowAddr = 0x00;
        result.whoHighAddr = 0x00;
        result.ackLowAddr = readWhoNoAddressBias(ISM330BX_I2C_ADDR_LOW, result.whoLowAddr);
        result.ackHighAddr = readWhoNoAddressBias(ISM330BX_I2C_ADDR_HIGH, result.whoHighAddr);
        return result;
    };

    const PassResult drivenLow = runPass(LOW);
    const PassResult drivenHigh = runPass(HIGH);

    pinMode(IMU_MISO, INPUT);

    dbgLog("[IMU][SPIDBG] SDO drive LOW:  addr 0x6A ack=%d WHO=0x%02X | addr 0x6B ack=%d WHO=0x%02X",
           drivenLow.ackLowAddr, drivenLow.whoLowAddr,
           drivenLow.ackHighAddr, drivenLow.whoHighAddr);
    dbgLog("[IMU][SPIDBG] SDO drive HIGH: addr 0x6A ack=%d WHO=0x%02X | addr 0x6B ack=%d WHO=0x%02X",
           drivenHigh.ackLowAddr, drivenHigh.whoLowAddr,
           drivenHigh.ackHighAddr, drivenHigh.whoHighAddr);

    const bool lowSelects6A = drivenLow.ackLowAddr && drivenLow.whoLowAddr == WHO_AM_I_EXPECTED && !drivenLow.ackHighAddr;
    const bool highSelects6B = drivenHigh.ackHighAddr && drivenHigh.whoHighAddr == WHO_AM_I_EXPECTED && !drivenHigh.ackLowAddr;
    const bool anyAck = drivenLow.ackLowAddr || drivenLow.ackHighAddr || drivenHigh.ackLowAddr || drivenHigh.ackHighAddr;

    if (!anyAck) {
        dbgLog("[IMU][SPIDBG] SDO/TA0 test inconclusive: IMU did not ACK I2C at either address");
        dbgLog("[IMU][SPIDBG] Cold power-cycle the whole board, then rerun spi-debug without IF_CFG");
        return;
    }

    if (lowSelects6A && highSelects6B) {
        dbgLog("[IMU][SPIDBG] SDO/TA0 follows IO37: MISO net continuity is likely OK");
    } else {
        dbgLog("[IMU][SPIDBG] SDO/TA0 does NOT follow IO37: suspect open IO37-SDO trace, bad U2 pin 1 solder, or SDO/TA0 stuck low");
    }
}

static void runCsGateTest() {
    dbgLog("[IMU][SPIDBG] CS gate test: I2C should ACK with CS high and disappear with CS low");

    uint8_t whoHigh = 0x00;
    uint8_t whoLow = 0x00;

    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);
    applyI2cAddressBias();
    delay(3);
    const bool ackWhenHigh = i2cReadReg(i2cAddr, reg::WHO_AM_I, whoHigh);

    digitalWrite(IMU_CS, LOW);
    pinMode(IMU_CS, OUTPUT);
#if IMU_SPI_CS_LOW_HOLD_MS > 0
    dbgLog("[IMU][SPIDBG] CS LOW HOLD: measure TP_CS1 / IMU CS now for %lu ms",
           static_cast<unsigned long>(IMU_SPI_CS_LOW_HOLD_MS));
    delay(IMU_SPI_CS_LOW_HOLD_MS);
#endif
    delay(3);
    const bool ackWhenLow = i2cReadReg(i2cAddr, reg::WHO_AM_I, whoLow);

    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);
    applyI2cAddressBias();

    dbgLog("[IMU][SPIDBG] CS HIGH: addr 0x%02X ack=%d WHO=0x%02X", i2cAddr, ackWhenHigh, whoHigh);
    dbgLog("[IMU][SPIDBG] CS LOW:  addr 0x%02X ack=%d WHO=0x%02X", i2cAddr, ackWhenLow, whoLow);

    if (ackWhenHigh && !ackWhenLow) {
        dbgLog("[IMU][SPIDBG] CS reaches the IMU: low disables I2C as expected");
    } else if (ackWhenHigh && ackWhenLow) {
        dbgLog("[IMU][SPIDBG] CS does NOT gate the IMU: suspect IO21-CS open, stuck high, or wrong CS pin");
    } else {
        dbgLog("[IMU][SPIDBG] CS test inconclusive: I2C did not ACK with CS high");
    }
}

static void runSpiScopeLoop() {
    dbgLog("[IMU][SPIDBG] scope loop: repeating slow bit-banged mode3 WHO reads");
    dbgLog("[IMU][SPIDBG] scope target: CS low, 16 clocks, MOSI first byte 0x8F, MISO second byte 0x71");
    dbgLog("[IMU][SPIDBG] loop count=%u; set IMU_SPI_SCOPE_LOOP_COUNT=0 for endless UART-only probing",
           static_cast<unsigned>(IMU_SPI_SCOPE_LOOP_COUNT));

    uint32_t count = 0;
    while (IMU_SPI_SCOPE_LOOP_COUNT == 0 || count < IMU_SPI_SCOPE_LOOP_COUNT) {
        const BitBangSpiResult who = bitBangSpiReadReg(reg::WHO_AM_I, SPI_MODE3);
        if ((count % 10) == 0) {
            dbgLog("[IMU][SPIDBG] scope sample %lu raw=0x%04X WHO=0x%02X",
                   static_cast<unsigned long>(count), who.raw, who.value);
        }
        count++;
        delay(100);
    }
    dbgLog("[IMU][SPIDBG] scope loop complete; continuing boot so OTA can recover");
}
#endif

#if IMU_SPI_DEBUG
static bool runSpiDebugPass(bool i2cAlive) {
    logSpiDebugHeader();
    bool i2cUsableForFallback = i2cAlive;

    pinMode(IMU_MISO, INPUT);
    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);
    delay(2);
    logSpiGpioState("idle before sweep");

    runHardwareSpiSweep("initial");
    runBitBangSpiSweep("initial");
    runSdoStrapDriveTest();
    runCsGateTest();

#if IMU_SPI_IFCFG_TEST
    dbgLog("[IMU][SPIDBG] IF_CFG test enabled; I2C may stop responding until reset or power-cycle");
    if (i2cAlive) {
        uint8_t ifCfgBefore = 0x00;
        if (i2cReadReg(i2cAddr, reg::IF_CFG, ifCfgBefore)) {
            const uint8_t ifCfgAfter = (ifCfgBefore | IF_CFG_I2C_I3C_DISABLE) & ~IF_CFG_SIM_3WIRE;
            dbgLog("[IMU][SPIDBG] IF_CFG before=0x%02X, writing=0x%02X", ifCfgBefore, ifCfgAfter);
            if (i2cWriteReg(i2cAddr, reg::IF_CFG, ifCfgAfter)) {
                i2cUsableForFallback = false;
                delay(5);
                runHardwareSpiSweep("after I2C/I3C disable");
                runBitBangSpiSweep("after I2C/I3C disable");
                dbgLog("[IMU][SPIDBG] I2C fallback disabled for this boot; power-cycle to restore it");
            } else {
                dbgLog("[IMU][SPIDBG] IF_CFG write failed");
            }
        } else {
            dbgLog("[IMU][SPIDBG] IF_CFG read failed; skipping interface-disable test");
        }
    } else {
        dbgLog("[IMU][SPIDBG] I2C was not alive; skipping interface-disable test");
    }
#endif

#if IMU_SPI_SCOPE_LOOP
    runSpiScopeLoop();
#endif

    logSpiDebugPowerCyclePrompt();
    imuSpi.begin(IMU_SCK, IMU_MISO, IMU_MOSI, -1);
    delay(2);
    logSpiGpioState("after debug restore");
    return i2cUsableForFallback;
}
#endif

static uint8_t regRead(uint8_t reg) {
    if (!useI2c) return spiRead(reg);

    uint8_t value = 0x00;
    if (!i2cReadReg(i2cAddr, reg, value)) {
        dbgLog("[IMU] I2C read failed: reg 0x%02X", reg);
    }
    return value;
}

static void regWrite(uint8_t reg, uint8_t val) {
    if (!useI2c) {
        spiWrite(reg, val);
        return;
    }
    if (!i2cWriteReg(i2cAddr, reg, val)) {
        dbgLog("[IMU] I2C write failed: reg 0x%02X", reg);
    }
}

static void regBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (!useI2c) {
        spiBurst(reg, buf, len);
        return;
    }
    if (!i2cReadBurst(i2cAddr, reg, buf, len)) {
        dbgLog("[IMU] I2C burst read failed: reg 0x%02X len %u", reg, len);
        for (uint8_t i = 0; i < len; i++) buf[i] = 0;
    }
}

bool imuInit() {
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_INT1, INPUT);
    pinMode(IMU_INT2, INPUT);

    pinMode(IMU_MISO, INPUT_PULLUP);
    delay(2);
    const int misoWithPullup = digitalRead(IMU_MISO);
    pinMode(IMU_MISO, INPUT_PULLDOWN);
    delay(2);
    const int misoWithPulldown = digitalRead(IMU_MISO);
    pinMode(IMU_MISO, INPUT);
    dbgLog("[IMU] MISO idle test: pullup=%d pulldown=%d", misoWithPullup, misoWithPulldown);

    const bool i2cAlive = runI2cAliveProbe();

    // Pass -1 for ss so the SPI driver never attaches IMU_CS to the hardware
    // peripheral's own CS line — CS is managed manually below via digitalWrite
    // across multiple transfer() calls, which requires it to stay a plain GPIO.
    imuSpi.begin(IMU_SCK, IMU_MISO, IMU_MOSI, -1);
    delay(10);

    bool i2cUsableForFallback = i2cAlive;
#if IMU_SPI_DEBUG
    i2cUsableForFallback = runSpiDebugPass(i2cAlive);
#endif

    const uint8_t whoMode3 = spiReadWithMode(reg::WHO_AM_I, SPI_MODE3);
    const uint8_t whoMode0 = spiReadWithMode(reg::WHO_AM_I, SPI_MODE0);
    dbgLog("[IMU] WHO probe: mode3=0x%02X mode0=0x%02X", whoMode3, whoMode0);
    if (whoMode3 == WHO_AM_I_EXPECTED) {
        useI2c = false;
        spiMode = SPI_MODE3;
        dbgLog("[IMU] Using SPI mode 3");
    } else if (whoMode0 == WHO_AM_I_EXPECTED) {
        useI2c = false;
        spiMode = SPI_MODE0;
        dbgLog("[IMU] Using SPI mode 0");
    } else if (i2cUsableForFallback) {
        useI2c = true;
        digitalWrite(IMU_CS, HIGH);
        pinMode(IMU_CS, OUTPUT);
        applyI2cAddressBias();
        dbgLog("[IMU] SPI did not answer; using I2C fallback at address 0x%02X", i2cAddr);
    } else {
        useI2c = false;
        spiMode = SPI_MODE3;
        if (i2cAlive) {
            dbgLog("[IMU] SPI did not answer and I2C fallback is disabled for this debug boot");
        }
    }

    // Software reset
    regWrite(reg::CTRL3, 0x01);
    delay(10);

    whoAmIByte = regRead(reg::WHO_AM_I);
    dbgLog("[IMU] WHO_AM_I = 0x%02X (expected 0x%02X)", whoAmIByte, WHO_AM_I_EXPECTED);

    if (whoAmIByte != WHO_AM_I_EXPECTED) {
        dbgLog("[IMU] WARNING: WHO_AM_I mismatch — check datasheet. Proceeding anyway.");
    }

    if (whoAmIByte == 0x00 || whoAmIByte == 0xFF) {
        dbgLog("[IMU] ERROR: SPI bus fault (0x00 or 0xFF). Check wiring.");
        initOk = false;
        return false;
    }

    // CTRL3: BDU=1 (block data update), IF_INC=1 (auto-increment for burst reads)
    regWrite(reg::CTRL3, 0x44);

    // CTRL1: accel @ 120 Hz, high-performance mode
    // bits[6:4] = 000 (high perf), bits[3:0] = 0110 (120 Hz)
    regWrite(reg::CTRL1, 0x06);

    // CTRL2: gyro @ 120 Hz, high-performance mode
    // bits[7,5:4] = 000 (high perf), bits[3:0] = 0110 (120 Hz)
    regWrite(reg::CTRL2, 0x06);

    // CTRL6: gyro full scale ±2000 dps
    // bits[3:0] = 0100
    regWrite(reg::CTRL6, 0x04);

    // CTRL8: accel full scale ±8g
    // bits[1:0] = 10
    regWrite(reg::CTRL8, 0x02);

    // INT1_CTRL: route DRDY_XL (bit0) and DRDY_G (bit1) to INT1 pin
    regWrite(reg::INT1_CTRL, 0x03);

    delay(5);

    // Verify register readbacks. If WHO_AM_I is good but these do not match,
    // the bus is writing poorly or the transaction format is wrong.
    const uint8_t ctrl3Check = regRead(reg::CTRL3);
    const uint8_t ctrl1Check = regRead(reg::CTRL1);
    const uint8_t ctrl2Check = regRead(reg::CTRL2);
    const uint8_t ctrl6Check = regRead(reg::CTRL6);
    const uint8_t ctrl8Check = regRead(reg::CTRL8);
    dbgLog("[IMU] CTRL3 readback = 0x%02X (expected 0x44)", ctrl3Check);
    dbgLog("[IMU] CTRL1 readback = 0x%02X (expected 0x06)", ctrl1Check);
    dbgLog("[IMU] CTRL2 readback = 0x%02X (expected 0x06)", ctrl2Check);
    dbgLog("[IMU] CTRL6 readback = 0x%02X (expected 0x04)", ctrl6Check);
    dbgLog("[IMU] CTRL8 readback = 0x%02X (expected 0x02)", ctrl8Check);

    initOk = true;
    dbgLog("[IMU] Init OK");
    return true;
}

bool imuAvailable() {
    if (useI2c) {
        const uint8_t status = regRead(reg::STATUS_REG);
        return (status & 0x03) != 0;  // bit0=XLDA, bit1=GDA
    }
    return digitalRead(IMU_INT1) == HIGH;
}

bool imuRead(ImuData& out) {
    if (!initOk) return false;

    uint8_t buf[6];

    // Gyro XYZ
    regBurst(reg::OUTX_L_G, buf, 6);
    out.gx = (int16_t)(buf[1] << 8 | buf[0]) * GYRO_SCALE;
    out.gy = (int16_t)(buf[3] << 8 | buf[2]) * GYRO_SCALE;
    out.gz = (int16_t)(buf[5] << 8 | buf[4]) * GYRO_SCALE;

    // Accel registers are ordered Z, Y, X starting at 0x28.
    regBurst(reg::OUTZ_L_A, buf, 6);
    out.az = (int16_t)(buf[1] << 8 | buf[0]) * ACCEL_SCALE;
    out.ay = (int16_t)(buf[3] << 8 | buf[2]) * ACCEL_SCALE;
    out.ax = (int16_t)(buf[5] << 8 | buf[4]) * ACCEL_SCALE;

    out.ts = millis();
    return true;
}

const char* imuStatus() {
    if (whoAmIByte == 0x00 || whoAmIByte == 0xFF) return "SPI_ERROR";
    if (!initOk) return "NOT_FOUND";
    return "OK";
}

uint8_t imuWhoAmI() {
    return whoAmIByte;
}
