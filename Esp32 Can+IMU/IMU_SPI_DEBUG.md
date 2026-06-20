# ISM330BX SPI Debug

Use this when I2C works but SPI reads `0x00` or `0xFF`.

## Build

OTA debug build:

```sh
pio run -e esp32-s3-wroom-1-ota-imu-spi-debug -t upload
```

Serial debug build:

```sh
pio run -e esp32-s3-wroom-1-imu-spi-debug -t upload
```

Scope-loop build:

```sh
pio run -e esp32-s3-wroom-1-ota-imu-spi-scope -t upload
```

## What It Runs

The debug firmware runs this sequence at boot:

1. I2C alive probe at `0x6A` and `0x6B`.
2. Hardware SPI `WHO_AM_I` sweep at 100 kHz, 400 kHz, and 1 MHz in modes 3 and 0.
3. Slow bit-banged SPI `WHO_AM_I` sweep in modes 3 and 0.
4. `SDO/TA0` strap test by driving ESP32 IO37 low/high and watching the I2C address move.
5. CS gate test by checking that I2C works with CS high and disappears with CS low.
6. A 15 second CS-low hold for measuring `TP_CS1` or the IMU CS pin with a multimeter.

The expected SPI read transaction is:

- CS low
- MOSI first byte: `0x8F`
- 16 clocks total
- MISO second byte: `0x71`

## Interpret Results

I2C alive:

- `0x6A ack=1 WHO=0x71` or `0x6B ack=1 WHO=0x71` means the IMU is powered and alive.
- No ACK at both addresses means cold power-cycle the board before trusting other results.

SDO/TA0 strap:

- LOW -> `0x6A`, HIGH -> `0x6B`: IO37 reaches IMU SDO/TA0.
- HIGH still stays at `0x6A`: suspect IO37-to-SDO open, bad U2 pin 1 solder, or SDO stuck low.

CS gate:

- CS HIGH ACKs and CS LOW does not ACK: CS reaches the IMU.
- CS HIGH ACKs and CS LOW still ACKs: CS is not reaching the IMU, stuck high, or firmware is using the wrong CS pin.

Current board result:

- IO37/SDO follows correctly.
- CS LOW still allows I2C ACK.
- `TP_CS1` was measured low during the 15 second CS-low hold.
- Leading fault is the `TP_CS1` / IMU pin 12 CS path, the IMU pin 12 solder joint, or the IMU CS input itself.

## Physical Checks

During the 15 second hold, measure:

- ESP32 IO21 if accessible
- `TP_CS1`
- IMU pin 12 CS side if accessible

Expected value during hold: near 0 V.

If `TP_CS1` is low but I2C still ACKs, inspect or reflow IMU pin 12.
If `TP_CS1` is not low, inspect IO21 routing, firmware pin mapping, or a short to 3.3 V.

For the current board, `TP_CS1` did measure low while I2C still ACKed. That points past ESP32 IO21 and toward the final connection into the IMU CS pin.

## Important

After any test that disables I2C/I3C, physically power-cycle the whole board. OTA upload or ESP32 reset may not reset the IMU.
