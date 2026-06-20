# RC Transmitter — Firmware README

Custom ESP32-S3 based RC transmitter built into a DJI Phantom 2 transmitter case. Dual radio support (nRF24L01+ and ESP-NOW), SPI monochrome LCD, PCF8575-based input/output expansion, and USB HID gamepad simulator mode.

## Hardware

| Component | Part | Notes |
|---|---|---|
| MCU | ESP32-S3-N16R8 | 16MB flash, 8MB OPI PSRAM, dual USB-C. Board mounted **reversed** in the case (physical left/right swapped) |
| Display | GMG12864-06D (ST7565R) | 128×64 monochrome, SPI, shares bus with nRF24 |
| Radio (long range) | nRF24L01+ | SPI, channel 100 |
| Radio (telemetry link) | built-in WiFi (ESP-NOW) | channel 1 |
| Battery monitor | Adafruit INA219 | I2C, address 0x40 |
| I/O expander | PCF8575 | I2C, address 0x20, shares bus with INA219 |
| Gimbals | 2× dual-axis analog | ADC1 only (safe to read with WiFi/ESP-NOW active) |
| Shoulder pots | 2× single-axis analog | CH9 / CH10 |
| Buzzer | piezo, active LOW | driven via PCF8575 output |
| Status LED | single LED, active LOW | driven via PCF8575 output |
| Vibromotor | mini vibration motor module (VIN/GND/IN) | driven via PCF8575 output |
| Switches | 2× 3-position, 2× 2-position | all wired through PCF8575 inputs |
| Nav buttons | OK + 4-way joystick (Up/Down/Left=Back/Right) | all wired through PCF8575 inputs |

⚠️ **PSRAM pins 35, 36, 37 must never be used on the N16R8 module** — touching them causes a TG1WDT crash loop.

## Pin Assignment

### Direct GPIO

| GPIO | Function |
|---|---|
| 1 | Left gimbal X — Rudder (CH4) |
| 2 | Left gimbal Y — Throttle (CH3) |
| 4 | Right gimbal X — Aileron (CH1) |
| 5 | Right gimbal Y — Elevator (CH2) |
| 6 | Left shoulder pot (CH9) |
| 7 | SPI MOSI — shared by nRF24 + display |
| 8 | Right shoulder pot (CH10) |
| 9 | SPI SCK — shared by nRF24 + display |
| 10 | nRF24 CS |
| 11 | nRF24 CE |
| 12 | nRF24 IRQ |
| 13 | I2C SDA — shared by INA219 + PCF8575 |
| 14 | I2C SCL — shared by INA219 + PCF8575 |
| 15 | SPI MISO — nRF24 only (display doesn't need it) |
| 16 | Display CS |
| 20 | Display backlight (PWM) |
| 45 | Display DC |
| 17, 18, 19, 21, 39, 40, 41, 42, 46, 47, 48 | **free** — previously used for buttons/switches/LED before PCF8575 migration |

### PCF8575 expander (I2C address 0x20)

| Bit | Module pin | Direction | Function |
|---|---|---|---|
| 0 | P0 | input | OK button |
| 1 | P1 | input | Joystick Up |
| 2 | P2 | input | Joystick Down |
| 3 | P3 | input | Joystick Left / Back |
| 4 | P4 | input | Joystick Right |
| 5 | P5 | input | 3-pos switch A — Flight Mode (CH5) pin A |
| 6 | P6 | input | 3-pos switch A — Flight Mode (CH5) pin B |
| 7 | P7 | input | 2-pos switch — ARM (CH7) |
| 8 | P10 | input | 2-pos switch — RTH (CH8) |
| 9 | P11 | input | 3-pos switch B — Camera (CH6) pin A |
| 10 | P12 | input | 3-pos switch B — Camera (CH6) pin B |
| 11 | P13 | output | Status LED (active LOW) |
| 12 | P14 | output | Buzzer (active LOW) |
| 13 | P15 | output | Vibromotor IN (active LOW) |
| 14, 15 | P16, P17 | — | spare |

Note: module silkscreen uses port-based labels (P0–P7, P10–P17); the bit index above is the value used in firmware (`pcf8575.digitalRead/Write(bit, ...)`).

## Channel Map

| CH | Function | Source |
|---|---|---|
| 1 | Aileron | right gimbal X |
| 2 | Elevator | right gimbal Y |
| 3 | Throttle | left gimbal Y |
| 4 | Rudder | left gimbal X |
| 5 | Flight Mode | 3-pos switch A |
| 6 | Camera / Aux | 3-pos switch B |
| 7 | ARM | 2-pos switch |
| 8 | RTH | 2-pos switch |
| 9 | Pot A | left shoulder pot |
| 10 | Pot B | right shoulder pot |

## Features

- **Model profiles** — 4 storable slots (name, type, radio mode, calibration, expo/rate, ESP-NOW peer), persisted to NVS. Copy and reset supported from the Models screen.
- **Gimbal calibration wizard** — 3-step guided process (idle → center → extreme).
- **Axis reverse** — per-channel toggle, stored per model.
- **Expo & Rates** — FlySky-style screen with live curve graph; per-channel expo (0–100%) and rate (50–100%), plus selectable TX update rate (25/50/100 Hz).
- **Dual radio** — nRF24L01+ (long range, one-way) and ESP-NOW (bidirectional, supports telemetry and USB HID passthrough). Switchable per model.
- **ESP-NOW pairing** — broadcast-based handshake; receiver MAC is saved per model and automatically reloaded/re-registered on boot and model switch.
- **Telemetry** — RSSI, link quality, RX battery voltage, RX arm state received back from the receiver over ESP-NOW; shown in the status bar and a dedicated Telemetry screen.
- **USB HID gamepad mode** — TX can act as a USB joystick/gamepad directly over its native USB-C port for simulator use.
- **Shared packet structs** — channel data and pairing packets use identical `struct` definitions (`ChannelPkt`, `EspNowDataPkt`, `EspNowPkt`, `TelemetryPkt`) on TX and RX, copy-pasted byte-for-byte to eliminate encoding mismatches. No manual byte-packing.
- **Home screen** — Radiomaster-Pocket-style stick indicators (3px fill bar for throttle, dot-sliders for elevator/rudder/aileron), telemetry column, model icon, pot bars, switch states (shown by channel number, not function name, since function varies per model type), low-battery and throttle-high warning overlays.
- **Non-blocking buzzer + vibromotor** — both run through independent state machines (`buzzTick()` / `vibroTick()`), never stall the main loop.
  - Buzzer: menu navigation, battery alerts, throttle alerts.
  - Vibromotor: battery alerts, throttle alerts, connection-lost alert (vibro only, no buzzer, for connection loss).
- **Backlight auto-dim** — dims after 30s idle, restores on any button press.
- **Boot animation** — expanding frame + title fade-in.
- **Throttle-low startup safety** — boot halts on a warning screen (with periodic beep) until the throttle stick is confirmed at minimum, preventing an accidental high-throttle arm right after power-on.
- **Hardware watchdog** — 3s timeout via `esp_task_wdt`; reboots automatically if the main loop ever locks up (e.g. an I2C/SPI hang) instead of leaving the TX frozen.
- **EMA-filtered ADC and battery readings** — gimbal/pot inputs and battery voltage use exponential moving average filtering instead of blocking multi-sample averaging, reducing per-cycle blocking time while keeping smooth values.
- **Packet sequence numbers** — `ChannelPkt` includes a wrapping 8-bit sequence counter, letting the receiver distinguish genuinely dropped packets from normal link latency (separate from the per-second link-quality estimate).
- **Smoothed ESP-NOW link quality** — uses the same 10-sample rolling window as the nRF24 path instead of a raw per-send 100/0 value.
- **Reduced NVS writes** — axis-reverse toggles and model-edit saves now write a single model slot instead of all 4, deferred until the screen is exited rather than on every keypress.
- **Flight timer** — counts up while the ARM channel is active, resets to 0:00 on disarm; shown on the home screen in the gap between the rudder and aileron sliders.
- **Configurable battery alert thresholds** — "Batt Warn" and "Batt Crit" voltage levels are now adjustable from the Settings menu (0.1V steps) instead of fixed at compile time; the two thresholds are kept from crossing each other.
- **Simple fixed-wing mixer presets** — selectable per model from the Model Edit screen (Name → Type → **Mixer**): None, Elevon, V-Tail, Flaperon. Mixing happens entirely on the TX by overwriting specific channel slots after expo/rate is applied — the RX has no awareness mixing occurred, it just outputs whatever lands in each channel. See "Mixer Presets" below for exact channel behaviour.
- **Scrollable menu**, **channel monitor**, **USB HID sim mode**, **About screen**.

## Mixer Presets

Mixing runs after per-axis expo/rate, overwriting specific channel slots in place. Select per-model via Model Edit → Mixer field.

| Mixer | Inputs combined | Output channels |
|---|---|---|
| None | — | passthrough, no change |
| Elevon | Aileron + Elevator stick | CH1 = elevonL, CH2 = elevonR |
| V-Tail | Elevator + Rudder stick | CH2 = ruddervatorL, CH4 = ruddervatorR |
| Flaperon | Aileron stick + Pot A (flap slider) | CH1 = aileronL, **CH6 = aileronR** (Camera slot repurposed) |

⚠️ **Flaperon repurposes CH6** (normally the Camera/Aux channel) as the second aileron servo output. Don't use the Flaperon mixer on a model where CH6 is wired to something else.

## Libraries Required

| Library | Notes |
|---|---|
| RF24 (nrf24/RF24) | v1.6.0+ |
| U8g2 | display driver |
| Adafruit INA219 | + Adafruit BusIO dependency |
| PCF8575 by Renzo Mischianti | I/O expander — [GitHub](https://github.com/xreef/PCF8575_library) |
| ESP32 core built-ins | `esp_now.h`, `esp_mac.h`, `Preferences.h`, `USB.h`, `USBHIDGamepad.h`, `Wire.h`, `SPI.h` |

## Arduino IDE Settings

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | Enabled |
| USB Mode | Hardware CDC and JTAG |
| Flash Size | 16MB |
| Partition Scheme | 16M Flash |
| PSRAM | OPI PSRAM |
| CPU Frequency | 240MHz |

## Known Constraints

- nRF24 and the display share the same SPI bus (MOSI=7, SCK=9) — display uses hardware SPI, no conflict.
- INA219 and PCF8575 share the same I2C bus (SDA=13, SCL=14) — no conflict since both are proper I2C devices with distinct addresses.
- GPIO 35/36/37 reserved for OPI PSRAM — never assign to peripherals.
- All custom enums/structs must be declared before any function definition in the `.ino` file (Arduino's auto-generated function prototypes are inserted immediately after the last `#include`, so any type used in a later function signature must already be visible at that point).

---
*This file is kept up to date by the assistant whenever firmware changes affect components, pins, or features. Last updated alongside firmware v4 (PCF8575 migration + vibromotor + shared packet structs).*
