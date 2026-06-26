# RC Transmitter/Receiver Project — Status Summary

> Source files are attached alongside this summary (`RC_TX_v4.ino`, `config.h`, `icons.h`, `rc_receiver_espnow.ino`) — those ARE the current full code. This file is the compact index/context, not a code dump, to keep token usage low in a fresh chat.

## 1. Hardware

**TX board:** ESP32-S3-N16R8 DevKitC (16MB flash, 8MB OPI PSRAM, dual USB-C), mounted **reversed** in a DJI Phantom 2 transmitter case (physical left/right swapped).
⚠️ GPIO 35/36/37 reserved for OPI PSRAM — never use, causes TG1WDT crash.

**RX board:** Tenstar Robot ESP32-C3 SuperMini (single-core RISC-V, 4MB flash). GPIO 8 = onboard blue LED (active LOW). GPIO 9 = BOOT button.

**Other parts:** GMG12864-06D display (ST7565R, SPI), nRF24L01+, Adafruit INA219, PCF8575 I/O expander, 2× dual-axis gimbals, 2× shoulder pots, 2× 3-pos switches, 2× 2-pos switches, piezo buzzer, status LED, mini vibromotor module — all TX-side. RX drives up to 6 PWM servos.

### TX Pinout (direct GPIO)
| GPIO | Function |
|---|---|
| 1/2/4/5 | Gimbals: LX=Rudder(CH4), LY=Throttle(CH3), RX=Aileron(CH1), RY=Elevator(CH2) |
| 6/8 | Pot A (CH9) / Pot B (CH10) |
| 7/9/15 | SPI MOSI/SCK (shared nRF24+display) / MISO (nRF24 only) |
| 10/11/12 | nRF24 CS/CE/IRQ |
| 13/14 | I2C SDA/SCL (shared INA219 + PCF8575, addr 0x40 / 0x20) |
| 16/45 | Display CS / DC |
| 20 | Display backlight PWM |
| 17,18,19,21,39,40,41,42,46,47,48 | **free** (post-PCF8575 migration) |

### TX Pinout (PCF8575, addr 0x20)
| Bit | Function | Bit | Function |
|---|---|---|---|
| 0-4 | OK, Up, Down, Back, Right | 9-10 | 3-pos Camera (CH6) A/B |
| 5-6 | 3-pos FlightMode (CH5) A/B | 11 | LED (output) |
| 7 | ARM switch (CH7) | 12 | Buzzer (output) |
| 8 | RTH switch (CH8) | 13 | Vibromotor (output) |

### RX Pinout
| GPIO | Function |
|---|---|
| 1,2,3,4,5,7 | Servo outputs (default `CH_MAP = {0,1,2,3,6,7}` → AIL,ELE,THR,RUD,ARM,RTH) |
| 8 | Status LED (active LOW) |
| 9 | BOOT (pairing trigger) |
| 3 | optional RX battery sense (divider 100k+47k), `RX_VBAT_PIN -1` to disable |

### Channel map (both boards)
CH1 Aileron, CH2 Elevator, CH3 Throttle, CH4 Rudder, CH5 FlightMode, CH6 Camera, CH7 ARM, CH8 RTH, CH9 PotA, CH10 PotB.

## 2. Completed Features

**TX:** 4 model profiles (NVS), gimbal calibration wizard, axis reverse, Expo/Rate screen with FlySky-style live curve graph + selectable TX freq (25/50/100Hz), dual radio (nRF24 + ESP-NOW, switchable per model), ESP-NOW pairing (MAC saved per model, auto-restored on boot/model-switch), telemetry display (RSSI/LQ/RX voltage/ARM from RX), USB HID gamepad sim mode, model copy/reset, fixed-wing mixer presets (None/Elevon/V-Tail/Flaperon — Flaperon repurposes CH6), flight timer (resets on disarm), configurable battery alert thresholds (Settings menu), non-blocking buzzer+vibromotor (vibro = battery/throttle/**connection-lost-only**, buzzer = menu+battery+throttle), backlight auto-dim, boot logo splash, throttle-low startup safety lockout, hardware watchdog (`esp_task_wdt`), EMA-filtered ADC + battery readings, packet sequence numbers for loss detection, smoothed ESP-NOW link quality, reduced NVS writes (single-slot saves). Custom icons in `icons.h` (logo, plane/sedan/heli/drone).

**RX:** ESP-NOW pairing (broadcast handshake), failsafe (500ms timeout, single-source-of-truth state machine), telemetry send-back every 500ms, configurable `CH_MAP[]`, rate-limited+threshold-gated servo writes (50Hz, skips redundant writes), optional EMA servo smoothing, watchdog, automatic ESP-NOW reconnect (boot retry w/ backoff + runtime total-silence recovery), startup servo-attach validation with LED fail-count indicator, packet-loss stats (`droppedPkts`).

**Shared:** `ChannelPkt`/`EspNowDataPkt`/`EspNowPkt`/`TelemetryPkt` structs byte-identical between TX/RX (no manual byte-packing, eliminates encoding mismatches).

## 3. Current Full Code

See attached files — **these are authoritative, do not reconstruct from memory:**
- `RC_TX_v4.ino` (~2370 lines) + `config.h` + `icons.h`
- `rc_receiver_espnow.ino` (~580 lines)
- Full READMEs with complete pin tables already exist in each folder — refer to those for anything not covered above.

⚠️ **Arduino preprocessor rule (recurring bug source):** ALL enums/structs/global vars used in ANY function signature must be declared before the first function definition in the file (Arduino auto-inserts prototypes right after the last `#include`). When adding new types, place them in the TYPES block near the top, before `pcfRead()`/PCF8575 driver code.

⚠️ File uses **CRLF line endings** (Windows/Arduino IDE) — preserve when editing.

## 4. Known Bugs / Open Items

- **No known open bugs** as of last session (mixerTypeNames missing-declaration bug was found and fixed).
- **Pending discussion, not started:** power button module (self-latching power switch, hardware solves TX on/off cleanly — preferred direction over deep sleep) and/or ESP32 deep sleep (rejected as primary approach due to INA219/nRF24 residual draw + RTC-GPIO restriction conflicting with PCF8575 buttons).

### Suggested next features (not yet implemented, prioritized)
1. Trim buttons (easy)
2. Range test mode (easy)
3. Stick mode switching 1/2/3/4 (easy)
4. Sub-trim screen (easy)
5. Per-channel endpoints (easy)
6. TX-pushed configurable failsafe values to RX (easy-medium)
7. Logical switches, EdgeTX-style (medium)
8. Flight-mode presets (medium)
9. OTA firmware update via WiFi AP (medium)
10. Channel order remap (medium)
11. Telemetry logging, PC companion tool, custom point-based curves (harder)
12. Power button module integration (hardware decision pending)
