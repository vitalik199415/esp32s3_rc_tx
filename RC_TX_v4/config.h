#pragma once
// =============================================================================
// RC TRANSMITTER v3.0 — config.h
// Board : ESP32-S3-N16R8 DevKitC (16MB Flash, 8MB OPI PSRAM, dual USB-C)
// Mount : Board REVERSED in case — physical left/right swapped
//
// !! PSRAM pins 35, 36, 37 MUST NOT be used on N16R8 — causes TG1WDT crash !!
// =============================================================================

#define FW_VERSION   "4.0.0"
#define MODEL_NAME   "Model 1"

// ── I2C ───────────────────────────────────────────────────────────────────────
#define PIN_I2C_SDA     13
#define PIN_I2C_SCL     14

// ── SPI — nRF24L01+ ──────────────────────────────────────────────────────────
#define PIN_SPI_MOSI    7
#define PIN_SPI_MISO    15
#define PIN_SPI_SCK     9
#define PIN_NRF_CS      10
#define PIN_NRF_CE      11
#define PIN_NRF_IRQ     12

// ── Analog inputs (ADC1 only — safe with WiFi/ESP-NOW) ───────────────────────
#define PIN_JOY_RX      4    // Right gimbal X — Aileron  CH1
#define PIN_JOY_RY      5    // Right gimbal Y — Elevator CH2
#define PIN_JOY_LX      1    // Left  gimbal X — Rudder   CH4
#define PIN_JOY_LY      2    // Left  gimbal Y — Throttle CH3
#define PIN_POT_A       6    // Left  shoulder pot        CH9
#define PIN_POT_B       8    // Right shoulder pot        CH10

// ── PCF8575 I2C GPIO expander ─────────────────────────────────────────────────
// Shares I2C bus with INA219 (SDA=13, SCL=14) — address 0x20 (A0=A1=A2=GND)
// INPUTS (active LOW — connect each switch/button between PCF pin and GND):
// OUTPUTS (active LOW — LED/buzzer between PCF pin and VCC via resistor)
#define PCF8575_ADDR    0x20

// ── PCF8575 inputs (P0-P10) ───────────────────────────────────────────────────
#define PCF_BTN_OK      0    // P0  — OK button
#define PCF_BTN_UP      1    // P1  — Joystick Up
#define PCF_BTN_DOWN    2    // P2  — Joystick Down
#define PCF_BTN_BACK    3    // P3  — Joystick Left / Back
#define PCF_BTN_RIGHT   4    // P4  — Joystick Right
#define PCF_SW3A_A      5    // P5  — 3-pos Flight Mode pin A
#define PCF_SW3A_B      6    // P6  — 3-pos Flight Mode pin B
#define PCF_SW_ARM      7    // P7  — 2-pos ARM switch
#define PCF_SW_RTH      8    // P8  — 2-pos RTH switch
#define PCF_SW3B_A      9    // P9  — 3-pos Camera pin A
#define PCF_SW3B_B      10   // P10 — 3-pos Camera pin B

// ── PCF8575 outputs (P11-P12, active LOW) ────────────────────────────────────
// LED:    connect between 3.3V and P11 via 330Ω resistor (P11 LOW = LED on)
// Buzzer: connect between 3.3V and P12 via 100Ω resistor or direct (P12 LOW = on)
#define PCF_LED         11   // P11 — status LED (active LOW)
#define PCF_BUZZER      12   // P12 — buzzer (active LOW)
#define PCF_VIBRO       13   // P15 — vibromotor module IN pin (active LOW)
// P14-P15 spare (bits 14-15)

// ── Freed GPIOs (now free for future use) ────────────────────────────────────
// 17 18 20 21 39 40 41 42 46 47 48

// ── Outputs ───────────────────────────────────────────────────────────────────
#define PIN_LCD_BL      20   // backlight PWM — GPIO 20

// ── Display GMG12864-06D (ST7565R, SPI) ──────────────────────────────────────
// Shares SPI bus with nRF24 (MOSI=GPIO7, SCK=GPIO9)
// CS and DC repurposed from LED_GREEN and LCD_BL
#define PIN_LCD_CS      16   // SPI chip select 
#define PIN_LCD_DC      45   // data/command
// RST: tie to 3.3V on display module 
#define LCD_CONTRAST    58   // ST7565R: start at 58 (0x3A), tune if too dim/dark

// ── INA219 ────────────────────────────────────────────────────────────────────
#define INA219_ADDR     0x40

// ── Battery (2S LiPo) ────────────────────────────────────────────────────────
#define BATT_FULL_V     8.4f
#define BATT_WARN_V     7.0f
#define BATT_CRIT_V     6.6f

// ── Radio ─────────────────────────────────────────────────────────────────────
#define RF_CHANNEL      100
#define RF_PAYLOAD_SIZE 20
#define ESPNOW_CHANNEL  1

// ── Channels ──────────────────────────────────────────────────────────────────
#define CHANNEL_COUNT   10
#define CH_MID          1500
#define CH_MIN          1000
#define CH_MAX          2000
#define CH_THROTTLE     2
#define CH_THROTTLE_MIN 1000

// ── ADC ───────────────────────────────────────────────────────────────────────
#define ADC_SAMPLES     8
#define ADC_MAX         4095
