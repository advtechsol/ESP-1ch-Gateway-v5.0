# ESP Single Channel LoRaWAN Gateway

This repository hosts the firmware we run on ESP8266 and ESP32 boards to build a light-weight, single-channel LoRaWAN gateway. The focus of this fork is on practical deployment: minimal setup, stable field behaviour, and a web interface you can actually use when the node is strapped to a wall.

## Highlights
- ESP8266 and ESP32 builds share the same code base (`ESP-sc-gway`).
- Supports RFM95/SX1276 radios with configurable spreading factor and channel activity detection.
- Integrated web console for on-device configuration, status, and basic packet inspection.
- Optional OLED status display, OTA updates, and statistics logging for long-running installs.
- Base64 helpers and networking stack kept in-tree so the sketch compiles cleanly with current Arduino cores.

## Hardware We Use
- ESP8266 Wemos D1 mini or ESP32-based TTGO-style boards.
- HopeRF RFM95W (868/915 MHz) breakout or equivalent SX1276 module.
- Level shifting is not required; keep wiring short and clean.
- Optional: SSD1306/SH1106 OLED on I²C, GPS module for gateway timestamping, external antenna.

## Repository Layout
- `ESP-sc-gway/ESP-sc-gway.ino` – main sketch entry point.
- `ESP-sc-gway/_*.ino` – feature blocks split out for readability (state machine, web server, OTA, etc.).
- `ESP-sc-gway/ESP-sc-gway.h` – build-time configuration flags and defaults.
- `ESP-sc-gway/loraModem.h` – pin mapping and LoRa radio setup.
- `ESP-sc-gway/base64_compat.h` – bundled helpers so downstream payloads decode without external libraries.

## Quick Start
1. Install the ESP8266 or ESP32 board packages in the Arduino IDE (or use `arduino-cli`).
2. Clone this repo and open `ESP-sc-gway/ESP-sc-gway.ino`.
3. Edit `ESP-sc-gway/ESP-sc-gway.h`:
   - Set Wi-Fi credentials in the `wpas` array.
   - Choose your pin mapping with `_PIN_OUT` or customise `loraModem.h`.
   - Set `_LFREQ`, `_SPREADING`, and other radio defaults if you need different regional settings.
4. (Optional) Enable extras like OTA (`A_OTA`), OLED support (`USE_OLED` + `OLED_TYPE`), or strict single-channel operation (`_STRICT_1CH`). When using the bundled Heltec-style SSD1306 driver (`OLED_TYPE == 3`), no external libraries are required.
5. Select the correct board/port and upload. Watch the serial console (115200 baud) for first boot status and the assigned IP address.
6. Point a browser at `http://<gateway-ip>/` to finish runtime configuration and monitor packets.

## Runtime Configuration
- Web interface exposes debug levels, radio parameters, server targets, and statistics.
- Configuration persists in SPIFFS; `WlanReadWpa()` reloads Wi-Fi profiles after reboot.
- OTA updates are available once enabled and a first USB flash has been done.

## Troubleshooting
- If the radio never leaves CAD, recheck DIO wiring and verify `_CAD` matches your setup.
- Heap pressure below ~18 KB can cause random behaviour; disable unused features in `ESP-sc-gway.h`.
- Serial logs prefixed with `T` come from the TX/RX path (`_txRx.ino`); enable higher debug levels for deeper traces.
- Use the web console to reset stats or trigger rejoin tests before re-flashing.

## Useful Links
- LoRaWAN regional parameters: https://lora-alliance.org/lorawan-for-developers/
- Arduino board support packages: https://github.com/espressif/arduino-esp32 and https://github.com/esp8266/Arduino

Happy hacking.
