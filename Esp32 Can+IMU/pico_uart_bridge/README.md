# Pico 2 USB-UART Bridge

This turns a Raspberry Pi Pico 2 into a simple 3.3 V USB-to-UART bridge for
ESP32-S3 bring-up.

The bridge is fixed at 115200 baud. That is the ESP32-S3 ROM boot log speed and
is also usable for esptool uploads if PlatformIO is kept at 115200.

## Wiring

Use only 3.3 V UART logic.

```text
Pico 2 GND  -> board GND / TP_GND1
Pico 2 GP4  -> ESP32 RXD0 / U1 pad 36 / GPIO44
Pico 2 GP5  -> ESP32 TXD0 / U1 pad 37 / GPIO43
```

Cross the UART lines:

```text
Pico TX -> ESP32 RXD0
Pico RX -> ESP32 TXD0
```

Do not connect Pico VBUS, VSYS, or 3V3 to the ESP32 board unless you are
intentionally powering the board that way. For normal debug, power the ESP32
board from its own supply and share only GND plus TX/RX.

## Flashing The Pico 2 — native firmware (recommended)

The `pico_sdk/` directory contains a native C firmware that compiles to a UF2
you drag-and-drop onto the Pico 2.  No CircuitPython runtime required.

### Prerequisites

* Pico SDK 2.x installed and `PICO_SDK_PATH` set in your shell.
* CMake ≥ 3.13 and the ARM GCC toolchain (`arm-none-eabi-gcc`).

### Build

```bash
cd pico_uart_bridge/pico_sdk

# Copy the SDK import helper (only needed once)
cp "$PICO_SDK_PATH/external/pico_sdk_import.cmake" .

mkdir build && cd build
cmake .. -DPICO_BOARD=pico2
make -j$(nproc)
```

The output is `pico_uart_bridge.uf2` inside the `build/` directory.

### Flash

1. Hold **BOOTSEL** on the Pico 2 and plug it in — it mounts as a USB drive.
2. Copy `build/pico_uart_bridge.uf2` to the drive.
3. The Pico reboots automatically and appears as **"Pico UART Bridge"** on macOS.

macOS will show a single serial port (e.g. `/dev/cu.usbmodem*`).  That is the
bridge port.

---

## Flashing The Pico 2 — CircuitPython (legacy)

1. Put CircuitPython for Raspberry Pi Pico 2 on the Pico 2.
2. Copy `circuitpython/boot.py` and `circuitpython/code.py` to the Pico's
   `CIRCUITPY` drive.
3. Unplug and replug the Pico.
4. macOS should show two Pico serial ports. Use the data CDC port for the bridge.

If you are unsure which `/dev/cu.*` entry is the bridge, run:

```bash
ls /dev/cu.*
```

Then unplug/replug the Pico and look for the new entries.

## ESP32-S3 Bootloader Test

Open a serial monitor at 115200 baud on the Pico bridge port, then tap RESET on
the ESP32 board. If the ESP32 is alive and TXD0 is connected, you should see ROM
boot text.

For upload, enter download mode manually:

```text
Hold BOOT
Tap RESET
Start upload
Release BOOT after esptool starts connecting
```

Use a slow upload speed:

```bash
pio run -t upload --upload-port /dev/cu.usbmodem*
```

If upload fails after connecting, add `upload_speed = 115200` to the ESP32
environment or temporarily upload with a config that sets 115200.

