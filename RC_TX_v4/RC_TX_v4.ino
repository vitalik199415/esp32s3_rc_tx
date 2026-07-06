// =============================================================================
// RC TRANSMITTER v3.1
// Features: NVS persistence, model profiles (4 slots), gimbal calibration,
//           scrollable menu, all inputs, nRF24 + ESP-NOW + USB HID
//
// Arduino IDE settings:
//   Board: ESP32S3 Dev Module | USB CDC On Boot: Enabled
//   USB Mode: Hardware CDC and JTAG | Flash: 16MB | Partition: 16M Flash
//   PSRAM: OPI PSRAM | CPU: 240MHz
//
// Libraries: nrf24/RF24@1.6.0  U8g2  Adafruit INA219  Adafruit BusIO
// =============================================================================

#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <Adafruit_INA219.h>
#include <RF24.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <USB.h>
#include <USBHIDGamepad.h>
#include "freertos/semphr.h"
#include "config.h"
#include "icons.h"

// =============================================================================
// TYPES — all at top (Arduino IDE preprocessor requirement)
// =============================================================================

enum RadioMode  : uint8_t { RADIO_NRF24 = 0, RADIO_ESPNOW = 1 };
enum PktType    : uint8_t { PTYPE_DATA=0x01, PTYPE_PAIR_REQ=0x10, PTYPE_PAIR_ACK=0x11, PTYPE_PAIR_DONE=0x12, PTYPE_TELEMETRY=0x20 };
enum PairState  : uint8_t { PAIR_IDLE, PAIR_SEARCHING, PAIR_SUCCESS, PAIR_TIMEOUT };

struct TelemetryPkt {
    PktType  type;       // PTYPE_TELEMETRY
    int8_t   rssi;       // dBm (negative, e.g. -65)
    uint8_t  lq;         // link quality 0-100%
    uint16_t rxVoltage;  // mV (0 if not measured)
    uint8_t  rxArmed;    // 1 if ARM channel > 1700
    int8_t   rxTemp;     // reserved / future
};

struct EspNowPkt {
    PktType type;
    uint8_t mac[6];
    uint8_t data[20]; // channel bytes when PTYPE_DATA
};
// Pairing packets only send type+mac, not the unused data[20] field
#define PAIR_PKT_SIZE (sizeof(PktType) + 6)

// Shared channel-data struct — MUST be byte-identical on TX and RX.
// Used as the raw nRF24 payload (no type byte needed — nRF24 only ever
// carries channel data on its dedicated pipe) and wrapped with a PktType
// byte for ESP-NOW (which multiplexes DATA/PAIR/TELEMETRY on one callback).
struct __attribute__((packed)) ChannelPkt {
    uint8_t  seq;           // increments every packet, wraps 0-255 — lets RX
                             // detect dropped packets (gap in sequence) vs.
                             // just slow/late arrival.
    uint16_t channels[10]; // CH1-CH10, raw microsecond values 1000-2000
};

struct __attribute__((packed)) EspNowDataPkt {
    PktType    type;       // always PTYPE_DATA
    ChannelPkt ch;
};
enum ModelType  : uint8_t { MODEL_AIRPLANE = 0, MODEL_CAR = 1, MODEL_HELI = 2, MODEL_DRONE = 3 };
// Simple fixed-wing mixer presets. Mixing happens entirely on TX — the RX
// just outputs whatever value lands in each channel slot, unaware mixing
// happened. NONE = passthrough (default for all non-fixed-wing models).
// ELEVON   : combines Aileron+Elevator stick into CH1/CH2 (elevonL/elevonR)
// VTAIL    : combines Elevator+Rudder stick into CH2/CH4 (ruddervatorL/R)
// FLAPERON : combines Aileron stick + Pot A (flap slider) into CH1/CH6
//            (aileronL/aileronR) — repurposes the Camera channel slot (CH6)
//            as the second aileron output for models using this mixer.
enum MixerType  : uint8_t { MIXER_NONE = 0, MIXER_ELEVON = 1, MIXER_VTAIL = 2, MIXER_FLAPERON = 3 };
enum BtnEvent   : uint8_t { BTN_NONE, BTN_UP, BTN_DOWN, BTN_BACK, BTN_RIGHT, BTN_OK };
enum CalibState : uint8_t { CALIB_IDLE, CALIB_CENTER, CALIB_EXTREME, CALIB_DONE };

enum UiState : uint8_t {
    UI_HOME = 0, UI_MENU,
    UI_MODELS, UI_MODEL_EDIT, UI_CHANNELS, UI_CH_MAP, UI_AXIS_REVERSE, UI_RATES, UI_RADIO, UI_ESPNOW_PAIR,
    UI_CALIBRATE, UI_SETTINGS, UI_TELEMETRY, UI_ABOUT
};

struct AxisCal {
    uint16_t minVal, centre, maxVal;
    bool     reversed;
};

struct ModelProfile {
    char      name[16];
    ModelType type;
    RadioMode radioMode;
    AxisCal   axCal[6];
    uint8_t   espnowPeer[6];
    // Expo: 0=linear, 1-100 = curve strength (per axis 0-3 = AIL/ELE/THR/RUD)
    uint8_t   expo[4];
    // Rate: 50-100% of full throw (per axis 0-3 = AIL/ELE/THR/RUD)
    uint8_t   rate[4];
    // TX send frequency in Hz: 25, 50, 100
    uint8_t   txFreqHz;
    MixerType mixerType; // fixed-wing mixer preset — see MixerType enum above
    // Channel remap: chRemap[i] = TX source index that feeds RX output slot i
    // Default identity (0→0 .. 9→9) — change to reroute any TX channel to any RX slot
    uint8_t   chRemap[10];
};

struct BtnState {
    uint8_t  pin;
    bool     lastRaw, pressed;
    uint32_t lastChange, pressStart, lastRepeat;
};

struct Setting {
    const char* label;
    int value, minVal, maxVal, step;
    uint8_t divisor; // 1 = show as integer, 10 = one decimal place (value/10.0)
};

// =============================================================================
// PCF8575 — using Renzo Mischianti's library
// Install via Library Manager: "PCF8575" by Renzo Mischianti
// GitHub: https://github.com/xreef/PCF8575_library
// =============================================================================
#include "PCF8575.h"
PCF8575 pcf8575(PCF8575_ADDR); // attaches to default Wire instance

static uint16_t pcfCache = 0xFFFF; // cached input state for pollButtons/readSwitches

// Refresh cache from PCF8575 — call periodically in loop
void pcfRead() {
    PCF8575::DigitalInput di = pcf8575.digitalReadAll();
    pcfCache = (uint16_t)(
        ((uint16_t)di.p0)        | ((uint16_t)di.p1  << 1)  |
        ((uint16_t)di.p2  << 2)  | ((uint16_t)di.p3  << 3)  |
        ((uint16_t)di.p4  << 4)  | ((uint16_t)di.p5  << 5)  |
        ((uint16_t)di.p6  << 6)  | ((uint16_t)di.p7  << 7)  |
        ((uint16_t)di.p8  << 8)  | ((uint16_t)di.p9  << 9)  |
        ((uint16_t)di.p10 << 10) | ((uint16_t)di.p11 << 11) |
        ((uint16_t)di.p12 << 12) | ((uint16_t)di.p13 << 13) |
        ((uint16_t)di.p14 << 14) | ((uint16_t)di.p15 << 15)
    );
}

// Set a single PCF output pin — library manages shadow register internally
void pcfSetPin(uint8_t bit, bool low) {
    pcf8575.digitalWrite(bit, low ? LOW : HIGH);
}

// Returns true if INPUT pin is LOW (active/pressed)
bool pcfPin(uint8_t bit) {
    return !(pcfCache & (1u << bit));
}

// LED, buzzer, vibromotor via PCF8575 (active LOW: LOW = on)
void ledOn()    { pcfSetPin(PCF_LED,    true);  }
void ledOff()   { pcfSetPin(PCF_LED,    false); }
void buzzOn()   { pcfSetPin(PCF_BUZZER, true);  }
void buzzOff()  { pcfSetPin(PCF_BUZZER, false); }
void vibroOn()  { pcfSetPin(PCF_VIBRO,  true);  }
void vibroOff() { pcfSetPin(PCF_VIBRO,  false); }

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void switchRadioMode(RadioMode m);
void handleInput(BtnEvent evt);
void saveSettings();
void loadSettings();
void beep(uint8_t count, uint16_t onMs, uint16_t offMs);

// =============================================================================
// PERIPHERALS
// =============================================================================
U8G2_ST7565_ERC12864_ALT_F_4W_SW_SPI* display = nullptr;
Adafruit_INA219*                      ina219  = nullptr;
RF24*                                 radio   = nullptr;
USBHIDGamepad*                        gamepad = nullptr;
Preferences                           prefs;

// =============================================================================
// CHANNELS
// =============================================================================
volatile uint16_t channels[CHANNEL_COUNT]; // written by Core 0 radioTask, read by Core 1 loop()

void channelsInit() {
    for (int i = 0; i < CHANNEL_COUNT; i++) channels[i] = CH_MID;
    channels[CH_THROTTLE] = CH_THROTTLE_MIN;
}

// =============================================================================
// BATTERY
// =============================================================================
bool  inaOk = false;
float battVoltage = 0, battCurrent = 0, battPower = 0;
// Configurable battery thresholds — defaults from config.h, adjustable via Settings
float battWarnV = BATT_WARN_V;
float battCritV = BATT_CRIT_V;

// Flight timer — counts up while ARM channel is active, resets to zero on disarm
uint32_t flightTimerStartMs   = 0;
uint32_t flightTimerElapsedMs = 0; // current run duration, ms — 0 when disarmed
bool     flightTimerRunning   = false;

void flightTimerTick() {
    bool armed = channels[6] > 1700;
    uint32_t now = millis();
    if (armed && !flightTimerRunning) {
        flightTimerRunning = true;
        flightTimerStartMs = now;
    } else if (!armed && flightTimerRunning) {
        flightTimerRunning   = false;
        flightTimerElapsedMs = 0; // reset on disarm
    }
    if (flightTimerRunning) {
        flightTimerElapsedMs = now - flightTimerStartMs;
    }
}

uint8_t battPercent() {
    return (uint8_t)(constrain(
        (battVoltage - battCritV) / (BATT_FULL_V - battCritV), 0.f, 1.f) * 100);
}

// =============================================================================
// MODELS
// =============================================================================
#define MODEL_COUNT 4

ModelProfile models[MODEL_COUNT];
uint8_t      activeModel = 0;

const char* modelTypeNames[] = { "Airplane", "Car", "Heli", "Drone" };
const char* mixerTypeNames[] = { "None", "Elevon", "V-Tail", "Flaperon" };

void modelsDefault() {
    const char* names[MODEL_COUNT] = { "Model 1","Model 2","Model 3","Model 4" };
    for (int m = 0; m < MODEL_COUNT; m++) {
        strncpy(models[m].name, names[m], 16);
        models[m].type      = MODEL_AIRPLANE;
        models[m].radioMode = RADIO_NRF24;
        memset(models[m].espnowPeer, 0xFF, 6);
        for (int i = 0; i < 6; i++) {
            models[m].axCal[i] = { 0, ADC_MAX/2, ADC_MAX, false };
        }
        models[m].axCal[1].reversed = true;
        for (int i = 0; i < 4; i++) { models[m].expo[i] = 0; models[m].rate[i] = 100; }
        models[m].txFreqHz  = 50;
        models[m].mixerType = MIXER_NONE;
        for (int i = 0; i < 10; i++) models[m].chRemap[i] = i; // identity — no remap
    }
}

void modelsLoad() {
    prefs.begin("rc_models", true);
    activeModel = prefs.getUChar("active", 0);
    for (int m = 0; m < MODEL_COUNT; m++) {
        char key[8];
        snprintf(key, sizeof(key), "mdl%d", m);
        if (prefs.getBytesLength(key) == sizeof(ModelProfile)) {
            prefs.getBytes(key, &models[m], sizeof(ModelProfile));
        }
    }
    prefs.end();
}

void modelSaveSlot(uint8_t idx) {
    // Save a single model slot — much faster than saving all 4
    prefs.begin("rc_models", false);
    char key[8];
    snprintf(key, sizeof(key), "mdl%d", idx);
    prefs.putBytes(key, &models[idx], sizeof(ModelProfile));
    prefs.end();
    yield(); // feed watchdog after NVS write
}

void modelsSave() {
    // Save active index + all slots
    prefs.begin("rc_models", false);
    prefs.putUChar("active", activeModel);
    prefs.end();
    yield();
    for (int m = 0; m < MODEL_COUNT; m++) {
        modelSaveSlot(m);
    }
}

void modelSaveActive() {
    // Save only the active model slot — use this after calibration/edits
    prefs.begin("rc_models", false);
    prefs.putUChar("active", activeModel);
    prefs.end();
    yield();
    modelSaveSlot(activeModel);
}

// =============================================================================
// RADIO STATE
// =============================================================================
RadioMode radioMode   = RADIO_NRF24;
bool      radioLinked = false;
uint8_t   linkQuality = 0;
bool      nrfOk       = false;
bool      espnowOk    = false;
bool      simModeOn   = false;
bool      espnowSendOk = false;

// Mutex protecting channels[] between Core 0 radioTask (writer) and Core 1 loop() (reader).
// NOTE: INA219 and PCF8575 both share Wire — INA219 reads in loop() at ~1Hz and PCF8575
// reads in radioTask at 100Hz. Collision probability is very low and smoothing absorbs
// any single corrupted sample. Add a Wire mutex here if this proves problematic.
static SemaphoreHandle_t sharedMux = nullptr;

// Telemetry received from RX (populated by onEspNowRecv)
struct TelemState {
    int8_t   rssi;
    uint8_t  lq;
    uint16_t rxVoltage;
    uint8_t  rxArmed;
    uint32_t lastRxMs;
    bool     fresh;
};
TelemState telem = {0, 0, 0, 0, 0, false};

// ESP-NOW pairing state variables
PairState  pairState        = PAIR_IDLE;
uint8_t    pairedMac[6]     = {0};
uint32_t   pairStartMs      = 0;
uint32_t   pairLastReqMs    = 0;
const uint32_t PAIR_TIMEOUT_MS   = 30000;
const uint32_t PAIR_REQ_INTERVAL = 500;
uint8_t   espnowPeer[6] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

// Apply active model settings
void modelApply() {
    switchRadioMode(models[activeModel].radioMode);
    // Load THIS model's saved peer MAC into the active global — not the
    // other way around. Previously this was reversed, which silently
    // overwrote each model's saved receiver with whatever the global
    // happened to hold (broadcast FF:FF:FF:FF:FF:FF on boot, or leftover
    // from whichever model was active before).
    memcpy(espnowPeer, models[activeModel].espnowPeer, 6);

    // Re-register the ESP-NOW peer with the now-correct MAC.
    // Skipped on the very first call during setup() since ESP-NOW isn't
    // initialized yet; espnowInit() will register the peer itself using
    // this freshly-loaded espnowPeer value.
    if (espnowOk) {
        if (esp_now_is_peer_exist(espnowPeer)) esp_now_del_peer(espnowPeer);
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, espnowPeer, 6);
        peer.channel = ESPNOW_CHANNEL; peer.encrypt = false;
        esp_now_add_peer(&peer);
    }
}

const char* radioModeName() { return radioMode == RADIO_NRF24 ? "nRF24" : "ESP-NOW"; }

const uint8_t TX_ADDR[6] = "RC_TX";
const uint8_t RX_ADDR[6] = "RC_RX";

// =============================================================================
// ADC INPUT
// =============================================================================
const uint8_t adcPins[6] = {
    PIN_JOY_RX, PIN_JOY_RY, PIN_JOY_LY, PIN_JOY_LX, PIN_POT_A, PIN_POT_B
};
const int chMap[6] = { 0, 1, 2, 3, 8, 9 };

// EMA (exponential moving average) filter — one ADC read per call instead
// of ADC_SAMPLES blocking reads, with smoothing equivalent via persistent state.
// alpha=0.3 gives similar settling behaviour to an 8-sample average but with
// 1/8th the blocking time per call.
static uint16_t emaState[6] = {0,0,0,0,0,0}; // indexed same as adcPins[]
static bool     emaInit[6]  = {false,false,false,false,false,false};

uint16_t readADCSmooth(uint8_t pin) {
    // Find which axis slot this pin belongs to (adcPins[] is small, linear scan is fine)
    uint8_t idx = 0;
    for (uint8_t i = 0; i < 6; i++) if (adcPins[i] == pin) { idx = i; break; }

    uint16_t raw = analogRead(pin);
    if (!emaInit[idx]) { emaState[idx] = raw; emaInit[idx] = true; return raw; }

    const float ALPHA = 0.3f;
    emaState[idx] = (uint16_t)(ALPHA * raw + (1.0f - ALPHA) * emaState[idx]);
    return emaState[idx];
}

uint16_t adcToUs(uint16_t raw, const AxisCal& cal) {
    const uint16_t DEAD = 40; // ADC counts dead zone around centre (~1% of range)
    int32_t offset = (int32_t)raw - (int32_t)cal.centre;
    if (abs(offset) < DEAD) return CH_MID; // snap to centre
    uint16_t us;
    if (raw < cal.centre) {
        us = map(raw, cal.minVal, cal.centre - DEAD, CH_MIN, CH_MID);
    } else {
        us = map(raw, cal.centre + DEAD, cal.maxVal, CH_MID, CH_MAX);
    }
    us = constrain(us, CH_MIN, CH_MAX);
    return cal.reversed ? (CH_MIN + CH_MAX - us) : us;
}

// Apply expo curve: expo 0=linear, 100=max curve
// Input/output in range -1.0..+1.0
float applyExpo(float x, uint8_t expoVal) {
    if (expoVal == 0) return x;
    float e = expoVal / 100.0f;
    // Standard RC expo: y = x*(1-e) + x^3*e
    return x * (1.0f - e) + x * x * x * e;
}

// Apply rate: scales throw symmetrically around centre
uint16_t applyRate(uint16_t us, uint8_t rateVal) {
    if (rateVal >= 100) return us;
    float centre = CH_MID;
    float scaled = centre + (us - centre) * (rateVal / 100.0f);
    return (uint16_t)constrain(scaled, CH_MIN, CH_MAX);
}

void readAnalog() {
    ModelProfile& m = models[activeModel];
    for (int i = 0; i < 6; i++) {
        uint16_t raw = readADCSmooth(adcPins[i]);
        uint16_t us  = adcToUs(raw, m.axCal[i]);
        // Apply expo and rate to first 4 axes (AIL/ELE/THR/RUD)
        if (i < 4) {
            // Normalise to -1..+1
            float norm = (us - CH_MID) / 500.0f;
            norm = applyExpo(norm, m.expo[i]);
            us   = (uint16_t)(CH_MID + norm * 500.0f);
            us   = constrain(us, CH_MIN, CH_MAX);
            us   = applyRate(us, m.rate[i]);
        }
        channels[chMap[i]] = us;
    }
    applyMixer(m.mixerType);
}

// Simple fixed-wing mixer presets. Runs after readAnalog() has populated
// channels[0..3] (AIL/ELE/THR/RUD, post-expo/rate) and channels[8..9]
// (PotA/PotB). Overwrites specific channel slots in-place with mixed
// values — the RX has no awareness this happened, it just outputs
// whatever value lands in each slot.
void applyMixer(MixerType mix) {
    if (mix == MIXER_NONE) return;

    auto norm  = [](uint16_t us) -> float { return (us - (float)CH_MID) / 500.0f; };
    auto toUs  = [](float n) -> uint16_t {
        n = constrain(n, -1.0f, 1.0f);
        return (uint16_t)(CH_MID + n * 500.0f);
    };

    if (mix == MIXER_ELEVON) {
        float ail = norm(channels[0]); // CH1 input
        float ele = norm(channels[1]); // CH2 input
        channels[0] = toUs(ele + ail); // elevonL -> CH1
        channels[1] = toUs(ele - ail); // elevonR -> CH2

    } else if (mix == MIXER_VTAIL) {
        float ele = norm(channels[1]); // CH2 input
        float rud = norm(channels[3]); // CH4 input
        channels[1] = toUs(ele + rud); // ruddervatorL -> CH2
        channels[3] = toUs(ele - rud); // ruddervatorR -> CH4

    } else if (mix == MIXER_FLAPERON) {
        float ail  = norm(channels[0]); // CH1 input
        float flap = norm(channels[8]); // PotA (CH9) as flap slider
        channels[0] = toUs(ail + flap); // aileronL  -> CH1
        channels[5] = toUs(-ail + flap); // aileronR -> CH6 (Camera slot repurposed)
    }
}

// =============================================================================
// SWITCH INPUT
// =============================================================================
uint16_t readSw3(uint8_t pA, uint8_t pB) {
    bool a = !digitalRead(pA), b = !digitalRead(pB);
    if  (a && !b) return CH_MIN;
    if (!a && !b) return CH_MID;
    if (!a &&  b) return CH_MAX;
    return CH_MID;
}
uint16_t readSw2(uint8_t p) { return !digitalRead(p) ? CH_MAX : CH_MIN; }

void readSwitches() {
    auto pcf3 = [](uint8_t bA, uint8_t bB) -> uint16_t {
        bool a = pcfPin(bA), b = pcfPin(bB);
        if  (a && !b) return (uint16_t)CH_MIN;
        if (!a && !b) return (uint16_t)CH_MID;
        if (!a &&  b) return (uint16_t)CH_MAX;
        return (uint16_t)CH_MID;
    };
    auto pcf2 = [](uint8_t bit) -> uint16_t {
        return pcfPin(bit) ? (uint16_t)CH_MAX : (uint16_t)CH_MIN;
    };
    channels[4] = pcf3(PCF_SW3A_A, PCF_SW3A_B);
    channels[5] = pcf3(PCF_SW3B_A, PCF_SW3B_B);
    channels[6] = pcf2(PCF_SW_ARM);
    channels[7] = pcf2(PCF_SW_RTH);
}

// =============================================================================
// NAV BUTTONS
// =============================================================================
// pin field repurposed as PCF bit index for v4
BtnState btns[] = {
    { PCF_BTN_UP,    true, false, 0, 0, 0 },
    { PCF_BTN_DOWN,  true, false, 0, 0, 0 },
    { PCF_BTN_BACK,  true, false, 0, 0, 0 },
    { PCF_BTN_RIGHT, true, false, 0, 0, 0 },
    { PCF_BTN_OK,    true, false, 0, 0, 0 },
};
const BtnEvent btnEvents[] = { BTN_UP, BTN_DOWN, BTN_BACK, BTN_RIGHT, BTN_OK };
const uint32_t DEBOUNCE_MS  = 10;  // 10ms — tighter debounce for PCF8575
const uint32_t REPEAT_DELAY = 300; // 300ms before repeat starts
const uint32_t REPEAT_MS    = 80;  // 80ms between repeats — snappier scrolling
BtnEvent       btnPending   = BTN_NONE;

void pollButtons() {
    uint32_t now = millis();
    for (uint8_t i = 0; i < 5; i++) {
        bool raw = pcfPin(btns[i].pin); // btns[i].pin = PCF bit index
        if (raw != btns[i].lastRaw) {
            btns[i].lastRaw    = raw;
            btns[i].lastChange = now;
        }
        if ((now - btns[i].lastChange) < DEBOUNCE_MS) continue;
        if (raw) {
            if (!btns[i].pressed) {
                // Fresh press — fire immediately
                btns[i].pressed    = true;
                btns[i].pressStart = now;
                btns[i].lastRepeat = now;
                if (btnPending == BTN_NONE) btnPending = btnEvents[i];
            } else {
                // Held — repeat fire for Up/Down/Right only
                BtnEvent ev = btnEvents[i];
                bool canRepeat = (ev == BTN_UP || ev == BTN_DOWN || ev == BTN_RIGHT);
                if (canRepeat && btnPending == BTN_NONE) {
                    uint32_t held = now - btns[i].pressStart;
                    if (held >= REPEAT_DELAY &&
                        (now - btns[i].lastRepeat) >= REPEAT_MS) {
                        btns[i].lastRepeat = now;
                        btnPending = ev;
                    }
                }
            }
        } else {
            btns[i].pressed    = false;
            btns[i].pressStart = 0;
            btns[i].lastRepeat = 0;
        }
    }
}

// =============================================================================
// BUZZER
// =============================================================================
// Non-blocking beep — fire and forget, runs via buzzTick() in main loop
struct BuzzQueue {
    uint8_t  count;
    uint8_t  done;
    bool     phase;   // true=on false=gap
    uint32_t endMs;
    uint16_t onMs;
    uint16_t offMs;
} bq = {0, 0, false, 0, 80, 80};

void beep(uint8_t count = 1, uint16_t onMs = 80, uint16_t offMs = 80) {
    bq.count  = count;
    bq.done   = 0;
    bq.onMs   = onMs;
    bq.offMs  = offMs;
    bq.phase  = true;
    bq.endMs  = millis() + onMs;
    buzzOn();
}

void buzzTick(uint32_t now) {
    if (bq.count == 0 || now < bq.endMs) return;
    if (bq.phase) {
        // End of ON phase
        buzzOff();
        bq.phase = false;
        bq.done++;
        if (bq.done >= bq.count) { bq.count = 0; return; }
        bq.endMs = now + bq.offMs;
    } else {
        // End of OFF gap — next beep
        buzzOn();
        bq.phase = true;
        bq.endMs = now + bq.onMs;
    }
}

// Non-blocking vibromotor pulses — same pattern as buzzTick().
// Used for battery/throttle/connection alerts only — NOT menu navigation,
// since vibro motors have slower mechanical response than a piezo buzzer.
struct VibroQueue {
    uint8_t  count;
    uint8_t  done;
    bool     phase;
    uint32_t endMs;
    uint16_t onMs;
    uint16_t offMs;
} vq = {0, 0, false, 0, 150, 100};

void vibrate(uint8_t count = 1, uint16_t onMs = 150, uint16_t offMs = 100) {
    vq.count  = count;
    vq.done   = 0;
    vq.onMs   = onMs;
    vq.offMs  = offMs;
    vq.phase  = true;
    vq.endMs  = millis() + onMs;
    vibroOn();
}

void vibroTick(uint32_t now) {
    if (vq.count == 0 || now < vq.endMs) return;
    if (vq.phase) {
        vibroOff();
        vq.phase = false;
        vq.done++;
        if (vq.done >= vq.count) { vq.count = 0; return; }
        vq.endMs = now + vq.offMs;
    } else {
        vibroOn();
        vq.phase = true;
        vq.endMs = now + vq.onMs;
    }
}

// =============================================================================
// RADIO — nRF24
// =============================================================================
bool nrfInit() {
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_NRF_CS);
    radio = new RF24(PIN_NRF_CE, PIN_NRF_CS);
    if (!radio->begin(&SPI)) { Serial.println("[NRF] Failed"); return false; }
    radio->setChannel(RF_CHANNEL);
    radio->setDataRate(RF24_250KBPS);
    radio->setPALevel(RF24_PA_HIGH);
    radio->setPayloadSize(RF_PAYLOAD_SIZE);
    radio->setAutoAck(true);
    radio->setRetries(3, 5);
    radio->openWritingPipe(TX_ADDR);
    radio->openReadingPipe(1, RX_ADDR);
    radio->stopListening();
    Serial.println("[NRF] OK");
    return true;
}

void nrfSend() {
    if (!nrfOk || radioMode != RADIO_NRF24) return;
    static uint8_t txSeq = 0;
    ChannelPkt pkt;
    pkt.seq = txSeq++;
    // chRemap[i] = source TX channel for RX output slot i (identity by default)
    for (int i = 0; i < 10; i++) pkt.channels[i] = channels[models[activeModel].chRemap[i]];
    bool ok = radio->write(&pkt, sizeof(ChannelPkt));
    static uint8_t lq[10]={0}; static uint8_t li=0;
    lq[li++%10] = ok?10:0;
    uint16_t s=0; for(int i=0;i<10;i++) s+=lq[i];
    linkQuality=s; radioLinked=linkQuality>50;
}

// =============================================================================
// RADIO — ESP-NOW
// =============================================================================
void onEspNowSend(const wifi_tx_info_t* info, esp_now_send_status_t status) {
    espnowSendOk = (status == ESP_NOW_SEND_SUCCESS);
}

// Forward declaration
void onPairAckReceived(const uint8_t* mac);

// Called when any ESP-NOW packet is received
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* rawData, int len) {
    if (len < 1) return;
    PktType type = (PktType)rawData[0];
    if (type == PTYPE_TELEMETRY && len >= (int)sizeof(TelemetryPkt)) {
        const TelemetryPkt* tp = (const TelemetryPkt*)rawData;
        telem.rssi      = tp->rssi;
        telem.lq        = tp->lq;
        telem.rxVoltage = tp->rxVoltage;
        telem.rxArmed   = tp->rxArmed;
        telem.lastRxMs  = millis();
        telem.fresh     = true;
        radioLinked     = true;
        linkQuality     = tp->lq;
        return;
    }
    if (type == PTYPE_PAIR_ACK && pairState == PAIR_SEARCHING && len >= 7) {
        // Receiver replied — save its MAC
        memcpy(pairedMac, rawData + 1, 6);
        onPairAckReceived(pairedMac);
    }
}

void onPairAckReceived(const uint8_t* mac) {
    // Save MAC to active model
    memcpy(models[activeModel].espnowPeer, mac, 6);
    memcpy(espnowPeer, mac, 6);
    // Re-register peer with actual MAC
    if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL; peer.encrypt = false;
    esp_now_add_peer(&peer);
    // Send PAIR_DONE to receiver
    EspNowPkt pkt;
    pkt.type = PTYPE_PAIR_DONE;
    uint8_t selfMac[6]; esp_read_mac(selfMac, ESP_MAC_WIFI_STA);
    memcpy(pkt.mac, selfMac, 6);
    esp_now_send(mac, (uint8_t*)&pkt, PAIR_PKT_SIZE);
    pairState = PAIR_SUCCESS;
    modelsSave();
    beep(3, 60, 40);
    Serial.printf("[PAIR] Paired with %02X:%02X:%02X:%02X:%02X:%02X\n",
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

bool espnowInit() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) { Serial.println("[ESPNOW] Failed"); return false; }
    esp_now_register_send_cb(onEspNowSend);
    esp_now_register_recv_cb(onEspNowRecv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, espnowPeer, 6);
    peer.channel = ESPNOW_CHANNEL; peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.println("[ESPNOW] OK");
    return true;
}

void espnowSend() {
    if (!espnowOk || radioMode != RADIO_ESPNOW) return;
    static uint8_t  espTxSeq = 0;
    static uint8_t  espLq[10] = {0};
    static uint8_t  espLi     = 0;

    EspNowDataPkt pkt;
    pkt.type     = PTYPE_DATA;
    pkt.ch.seq   = espTxSeq++;
    for (int i=0;i<10;i++) pkt.ch.channels[i] = channels[models[activeModel].chRemap[i]];
    esp_now_send(espnowPeer, (uint8_t*)&pkt, sizeof(EspNowDataPkt));

    // Smoothed link quality — same rolling-window approach as nRF24,
    // instead of a raw binary 100/0 per send which jumped around a lot.
    espLq[espLi++ % 10] = espnowSendOk ? 10 : 0;
    uint16_t sum = 0; for (int i=0;i<10;i++) sum += espLq[i];
    linkQuality = sum;
    radioLinked = linkQuality > 50;
}

void pairingStart() {
    pairState     = PAIR_SEARCHING;
    pairStartMs   = millis();
    pairLastReqMs = 0;
    memset(pairedMac, 0, 6);
    Serial.println("[PAIR] Searching for receiver...");
    beep(1, 80, 0);
}

void pairingTick() {
    if (pairState != PAIR_SEARCHING) return;
    uint32_t now = millis();
    if (now - pairStartMs > PAIR_TIMEOUT_MS) {
        pairState = PAIR_TIMEOUT;
        Serial.println("[PAIR] Timeout");
        beep(2, 200, 100);
        return;
    }
    if (now - pairLastReqMs < PAIR_REQ_INTERVAL) return;
    pairLastReqMs = now;
    // Broadcast PAIR_REQ
    EspNowPkt pkt;
    pkt.type = PTYPE_PAIR_REQ;
    uint8_t selfMac[6]; esp_read_mac(selfMac, ESP_MAC_WIFI_STA);
    memcpy(pkt.mac, selfMac, 6);
    uint8_t broadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_send(broadcast, (uint8_t*)&pkt, PAIR_PKT_SIZE);
}

void switchRadioMode(RadioMode m) {
    radioMode=m; radioLinked=false; linkQuality=0;
    models[activeModel].radioMode = m;
    Serial.printf("[RADIO] %s\n", radioModeName());
    beep(2,50,50);
}

// =============================================================================
// USB HID
// =============================================================================
// USB HID status
bool usbReady = false;

// Backlight auto-dim
static uint32_t lastActivityMs = 0;
static bool     dimmed         = false;
const  uint32_t DIM_TIMEOUT_MS = 30000; // 30s idle before dim
const  uint8_t  DIM_LEVEL      = 20;    // dim backlight level

void resetActivity() { lastActivityMs = millis(); }

// Battery buzzer state
static uint32_t lastBuzzMs       = 0;
static uint32_t lastConnLostMs   = 0; // throttles repeated lost-connection alerts
static bool     wasLinked         = false; // tracks link state for edge detection

// Throttle warning — shown on home screen
bool throttleWarning() {
    // For airplane/drone/heli: warn if throttle is not at minimum (bottom) when arming.
    // For car: throttle center (1500µs) = stopped, so warn if not near center instead.
    static bool wasArmed = false;
    bool armed = channels[6] > 1700;
    if (armed && !wasArmed) {
        wasArmed = true;
        bool isCar = (models[activeModel].type == MODEL_CAR);
        return isCar
            ? (abs((int)channels[2] - (int)CH_MID) > 100)  // car: must be near center
            : ((int)channels[2] > (int)(CH_THROTTLE_MIN + 100));  // others: must be at bottom
    }
    if (!armed) wasArmed = false;
    return false;
}

void simSend() {
    if (!simModeOn || !gamepad) return;

    // Mapping lambda scaling microsecond channel values to 8-bit signed range (-127 to 127)
    auto ax = [](uint16_t us) -> int8_t {
        return (int8_t)map((long)us, (long)CH_MIN, (long)CH_MAX, -127, 127);
    };

    // Initialize an empty 32-bit button bitmask
    uint32_t b = 0;

    // --- 1. DIGITAL SWITCH MAPPING (INPUT_PULLUP: LOW = Active) ---
    
    // CH7: ARM Switch (2-Position) -> Button 1
    if (pcfPin(PCF_SW_ARM)) b |= (1 << 0);

    // CH8: RTH Switch (2-Position) -> Button 2
    if (pcfPin(PCF_SW_RTH)) b |= (1 << 1);

    // CH5: 3-Position Switch A (Flight Mode) -> Button 3 (Position A) or Button 4 (Position B)
    if (pcfPin(PCF_SW3A_A)) {
        b |= (1 << 2); // Button 3 Active
    } else if (pcfPin(PCF_SW3A_B)) {
        b |= (1 << 3); // Button 4 Active
    }                  // Center position leaves both bits at 0

    // CH6: 3-Position Switch B (Camera/Aux) -> Button 5 (Position A) or Button 6 (Position B)
    if (pcfPin(PCF_SW3B_A)) {
        b |= (1 << 4); // Button 5 Active
    } else if (pcfPin(PCF_SW3B_B)) {
        b |= (1 << 5); // Button 6 Active
    }

    // --- 2. TRANSMIT VIA NATIVE ESP32 USBHIDGamepad::send() ---
    // chRemap applied: gamepad axis i reads from TX source channels[chRemap[i]]
    const uint8_t* r = models[activeModel].chRemap;
    bool ok = gamepad->send(
        ax(channels[r[0]]), // X  -> remapped CH1 (Aileron / Right Stick X)
        ax(channels[r[1]]), // Y  -> remapped CH2 (Elevator / Right Stick Y)
        ax(channels[r[3]]), // Z  -> remapped CH4 (Rudder / Left Stick X)
        ax(channels[r[2]]), // Rz -> remapped CH3 (Throttle / Left Stick Y)
        ax(channels[r[8]]), // Rx -> remapped CH9 (Left Shoulder Pot A)
        ax(channels[r[9]]), // Ry -> remapped CH10 (Right Shoulder Pot B)
        0,                  // Hat Switch (0 = Centered/None)
        b                   // 32-bit Button bitmask (Buttons 1 to 6 mapped)
    );

    usbReady = ok;

    // --- 3. HOST SAFETY WATCHDOG TIMER ---
    static uint32_t simStart = 0;
    if (simModeOn && simStart == 0) simStart = millis();
    
    if (!ok) {
        if (millis() - simStart > 5000) {
            simModeOn = false; 
            simStart = 0;
            Serial.println("[USB] No host detected — Sim mode disabled");
        }
    } else {
        simStart = 0; // Reset watchdog timer on successful transmission
    }
}



// =============================================================================
// SETTINGS PERSISTENCE
// =============================================================================
Setting settings[] = {
    { "Contrast",   60,  0, 255, 5,  1 },  // ST7565R: 58 default, tune as needed
    { "Backlight", 200,  0, 255, 5,  1 },
    { "Batt Warn",  70, 50,  84, 1, 10 },  // deci-volts: 70 = 7.0V
    { "Batt Crit",  66, 48,  80, 1, 10 },  // deci-volts: 66 = 6.6V
};
const uint8_t SETTINGS_COUNT = 4;

void saveSettings() {
    prefs.begin("rc_settings", false);
    prefs.putInt("contrast",  settings[0].value);
    prefs.putInt("backlight", settings[1].value);
    prefs.putInt("battWarn",  settings[2].value);
    prefs.putInt("battCrit",  settings[3].value);
    prefs.putUChar("radioMode", (uint8_t)radioMode);
    prefs.end();
    modelsSave(); // also save models when settings change
}

void loadSettings() {
    prefs.begin("rc_settings", true);
    settings[0].value = prefs.getInt("contrast",  255);
    settings[1].value = prefs.getInt("backlight", 200);
    settings[2].value = prefs.getInt("battWarn",  (int)(BATT_WARN_V * 10));
    settings[3].value = prefs.getInt("battCrit",  (int)(BATT_CRIT_V * 10));
    radioMode = (RadioMode)prefs.getUChar("radioMode", 0);
    prefs.end();
    battWarnV = settings[2].value / 10.0f;
    battCritV = settings[3].value / 10.0f;
}

void applySetting(uint8_t idx) {
    if (idx == 0) display->setContrast(settings[0].value);
    if (idx == 1) ledcWrite(PIN_LCD_BL, settings[1].value);
    if (idx == 2) battWarnV = settings[2].value / 10.0f;
    if (idx == 3) battCritV = settings[3].value / 10.0f;
    // Keep warn >= crit + 0.2V so the two thresholds can't cross/invert
    if (idx == 2 && settings[2].value <= settings[3].value + 2) {
        settings[2].value = settings[3].value + 2;
        battWarnV = settings[2].value / 10.0f;
    }
    if (idx == 3 && settings[3].value >= settings[2].value - 2) {
        settings[3].value = settings[2].value - 2;
        battCritV = settings[3].value / 10.0f;
    }
    saveSettings();
}

// =============================================================================
// CALIBRATION
// =============================================================================
CalibState calibState  = CALIB_IDLE;
uint32_t   calibStart  = 0;
uint16_t   calibMin[6], calibMax[6], calibCentre[6];
const uint32_t CALIB_EXTREME_MS = 4000; // 4s to move sticks to extremes

void calibBegin() {
    calibState = CALIB_CENTER;
    Serial.println("[CAL] Move sticks to CENTER, press OK");
}

void calibStep() {
    // Called every 20ms while calibrating — track min/max during extreme phase
    if (calibState != CALIB_EXTREME) return;
    for (int i = 0; i < 6; i++) {
        uint16_t raw = readADCSmooth(adcPins[i]);
        if (raw < calibMin[i]) calibMin[i] = raw;
        if (raw > calibMax[i]) calibMax[i] = raw;
    }
}

void calibConfirm() {
    if (calibState == CALIB_CENTER) {
        // Save centre values
        for (int i = 0; i < 6; i++) {
            calibCentre[i] = readADCSmooth(adcPins[i]);
            calibMin[i]    = calibCentre[i]; // init min/max to centre
            calibMax[i]    = calibCentre[i];
        }
        calibState = CALIB_EXTREME;
        calibStart = millis();
        Serial.println("[CAL] Move sticks to ALL extremes, press OK when done");
    } else if (calibState == CALIB_EXTREME) {
        // Apply calibration to active model only
        for (int i = 0; i < 6; i++) {
            models[activeModel].axCal[i].minVal  = max((int)calibMin[i]-20, 0);
            models[activeModel].axCal[i].maxVal  = min((int)calibMax[i]+20, ADC_MAX);
            models[activeModel].axCal[i].centre  = calibCentre[i];
        }
        calibState = CALIB_DONE;      // update state BEFORE saving (avoids blocking during active calib)
        Serial.println("[CAL] Saving calibration...");
        yield();                       // feed watchdog before NVS write
        modelSaveActive();             // save only active slot, not all 4
        yield();                       // feed watchdog after NVS write
        beep(3, 50, 50);
        Serial.println("[CAL] Calibration saved");
    }
}

// =============================================================================
// UI STATE
// =============================================================================
UiState uiState      = UI_HOME;
int8_t  menuCursor   = 0;
int8_t  menuOffset   = 0; // scroll offset for menu
int8_t  modelCursor  = 0;
int8_t  settingCursor   = 0;
bool    settingEditMode = false;

// Model editing
uint8_t editModelIdx  = 0;   // which model slot we're editing
uint8_t editField     = 0;   // 0=name, 1=type
bool    editFieldMode = false; // true = editing value, false = selecting field
uint8_t nameCursor    = 0;
bool    axisReverseDirty = false; // tracks unsaved axis-reverse toggles   // char position in name
// Character set for name editing
const char CHARSET[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_";
const uint8_t CHARSET_LEN = sizeof(CHARSET) - 1;

uint8_t charToIdx(char ch) {
    for (uint8_t i = 0; i < CHARSET_LEN; i++)
        if (CHARSET[i] == ch) return i;
    return 0;
}

const char* menuItems[] = {
    "Dashboard", "Models", "Channels", "Ch Map", "Axis Reverse", "Expo & Rates",
    "Radio", "Telemetry", "Calibrate", "Settings", "About"
};
const uint8_t MENU_COUNT     = 11;
const uint8_t MENU_VISIBLE   = 4; // items visible at once (with 6x10 font)

// =============================================================================
// DRAW HELPERS
// =============================================================================

// Draw inverted XBM bitmap (bitmaps have 0=dark, 1=light — opposite of U8g2 default)
void drawModelBitmap(uint8_t x, uint8_t y, const unsigned char* pgmBmp) {
    // Invert into stack buffer then draw normally
    uint8_t buf[72];
    for (uint8_t i = 0; i < 72; i++) buf[i] = ~pgm_read_byte(&pgmBmp[i]);
    display->drawXBMP(x, y, 24, 24, pgmBmp);
}

void drawStatusBar() {
    // White background
    display->setDrawColor(0);
    display->drawBox(0, 0, 128, 12);
    display->setDrawColor(1);
    display->drawFrame(0, 0, 128, 12);
    display->setFont(u8g2_font_5x7_tr);

    // Model name (left)
    display->drawStr(2, 9, models[activeModel].name);

    // Centre: radio mode + RSSI when telemetry fresh
    bool sbTelemFresh = telem.fresh && (millis()-telem.lastRxMs) < 2000;
    char centreStr[20];
    if (sbTelemFresh) {
        snprintf(centreStr, sizeof(centreStr), "%s %ddB", radioModeName(), (int)telem.rssi);
    } else {
        snprintf(centreStr, sizeof(centreStr), "%s", radioModeName());
    }
    uint8_t cw = display->getStrWidth(centreStr);
    display->drawStr((128-cw)/2, 9, centreStr);

    // Voltage right (replaces battery icon — cleaner, more readable)
    if (battVoltage > 0.1f) {
        char vbuf[8]; snprintf(vbuf, sizeof(vbuf), "%.2fV", battVoltage);
        uint8_t vw = display->getStrWidth(vbuf);
        display->drawStr(126 - vw, 9, vbuf);
    } else {
        display->drawStr(110, 9, "--V");
    }
}

void drawBreadcrumb(const char* t) {
    display->setFont(u8g2_font_5x7_tr);
    display->drawStr(2,8,"\x7f "); display->drawStr(10,8,t);
    display->drawHLine(0,10,128);
}

void drawBar(uint8_t x,uint8_t y,uint8_t w,uint8_t h,float pct) {
    display->drawFrame(x,y,w,h);
    uint8_t f=(uint8_t)(constrain(pct,0.f,1.f)*(w-2));
    if(f>0) display->drawBox(x+1,y+1,f,h-2);
}

// =============================================================================
// SCREENS
// =============================================================================
// ── Model type icons ─────────────────────────────────────────────────────────
void drawModelIcon(uint8_t x, uint8_t y, ModelType t) {
    switch (t) {
        case MODEL_AIRPLANE: drawModelBitmap(x, y, epd_bitmap_plane); break;
        case MODEL_CAR:      drawModelBitmap(x, y, epd_bitmap_sedan); break;
        case MODEL_HELI:     drawModelBitmap(x, y, epd_bitmap_heli); break;
        case MODEL_DRONE:    drawModelBitmap(x, y, epd_bitmap_drone); break;
    }
}

// ── Stick position box ────────────────────────────────────────────────────────
// xCh = horizontal channel index, yCh = vertical channel index (Y inverted)
void drawStickBox(uint8_t bx, uint8_t by, uint8_t sz, uint8_t xCh, uint8_t yCh) {
    display->drawFrame(bx, by, sz, sz);
    // Centre cross (subtle)
    uint8_t mid = sz / 2;
    display->drawPixel(bx + mid, by + mid - 1);
    display->drawPixel(bx + mid, by + mid + 1);
    display->drawPixel(bx + mid - 1, by + mid);
    display->drawPixel(bx + mid + 1, by + mid);
    // Dot position
    uint8_t dotX = (uint8_t)map(channels[xCh], CH_MIN, CH_MAX, 1, sz-2);
    uint8_t dotY = (uint8_t)map(channels[yCh], CH_MAX, CH_MIN, 1, sz-2); // inverted
    // 3x3 filled dot
    display->drawBox(bx + dotX - 1, by + dotY - 1, 3, 3);
}

// ── Switch state helpers ──────────────────────────────────────────────────────
uint8_t sw3pos(uint16_t val) {
    if (val < 1200) return 1;
    if (val < 1800) return 2;
    return 3;
}

void drawHome() {
    display->clearBuffer();
    drawStatusBar();

    // ==========================================================================
    // LAYOUT (128x64, status bar y=0..12):
    //   Throttle  : x=0..2,   y=13..54  — 3px fill-from-bottom
    //   Elevator  : x=125..127,y=13..54 — 3px dot slider (same style as H bars)
    //   Rudder    : x=0..49,  y=56..63  — horizontal dot slider
    //   Aileron   : x=78..127,y=56..63  — horizontal dot slider
    //   Telemetry : x=4..50,  y=13..54  — RSSI/LQ/RXV or "RX offline"
    //   Icon      : x=52..75, y=13..36  — 24x24 model bitmap
    //   Pot bars  : x=52..75, y=39..53  — CH9/CH10 one row each
    //   Switches  : x=77..122,y=13..54  — CH5-8 values
    // ==========================================================================

    const uint8_t VY=13, VH=42;
    const uint8_t HY=58, HH=5;
    const uint8_t HW=50; // horizontal bar width

    // ── THROTTLE — 3px wide, fill from bottom ────────────────────────────────
    display->drawFrame(0, VY, 3, VH);
    {
        float pct = constrain((float)(channels[2]-CH_MIN)/(CH_MAX-CH_MIN), 0.f, 1.f);
        uint8_t fill = (uint8_t)(pct * (VH-2));
        if (fill > 0) display->drawBox(1, VY+1+(VH-2-fill), 1, fill);
    }

    // ── ELEVATOR — 3px wide, dot slider (centre track + moving dot) ──────────
    display->drawFrame(125, VY, 3, VH);
    // faint centre track line (every other pixel for dotted effect)
    for (uint8_t yy = VY+1; yy < VY+VH-1; yy += 2)
        display->drawPixel(126, yy);
    // centre tick (solid)
    display->drawHLine(125, VY+VH/2, 3);
    {
        float pct = constrain((float)(channels[1]-CH_MIN)/(CH_MAX-CH_MIN), 0.f, 1.f);
        uint8_t pos = (uint8_t)((1.f-pct) * (VH-5)) + 1;
        display->drawBox(126, VY+pos, 1, 3);
    }

    // ── RUDDER — horizontal dot slider ───────────────────────────────────────
    display->drawFrame(0, HY, HW, HH);
    for (uint8_t xx = 2; xx < HW-1; xx += 2) display->drawPixel(xx, HY+HH/2);
    display->drawVLine(HW/2, HY, HH); // centre tick
    {
        float pct = constrain((float)(channels[3]-CH_MIN)/(CH_MAX-CH_MIN), 0.f, 1.f);
        uint8_t pos = (uint8_t)(pct * (HW-5)) + 1;
        display->drawBox(pos, HY+1, 3, HH-2);
    }

    // ── FLIGHT TIMER — fits the unused gap between rudder and aileron bars
    // (x=50..77, y=56..63, ~28px wide). Shows MM:SS, counts up while armed,
    // resets to 0:00 on disarm.
    {
        uint32_t totalSec = flightTimerElapsedMs / 1000;
        uint8_t  mins = totalSec / 60;
        uint8_t  secs = totalSec % 60;
        char tbuf[8]; snprintf(tbuf, sizeof(tbuf), "%d:%02d", mins, secs);
        display->setFont(u8g2_font_5x7_tr);
        uint8_t tw = display->getStrWidth(tbuf);
        // Centre in the 50..77 gap (width 27, centre x=63)
        display->drawStr(63 - tw/2, 62, tbuf);
    }

    // ── AILERON — horizontal dot slider ──────────────────────────────────────
    display->drawFrame(78, HY, HW, HH);
    for (uint8_t xx = 80; xx < 127; xx += 2) display->drawPixel(xx, HY+HH/2);
    display->drawVLine(78+HW/2, HY, HH); // centre tick
    {
        float pct = constrain((float)(channels[0]-CH_MIN)/(CH_MAX-CH_MIN), 0.f, 1.f);
        uint8_t pos = (uint8_t)(pct * (HW-5)) + 1;
        display->drawBox(78+pos, HY+1, 3, HH-2);
    }

    // ── TELEMETRY COLUMN (x=4..50) ───────────────────────────────────────────
    display->setFont(u8g2_font_4x6_tr);
    bool hmFresh = telem.fresh && (millis()-telem.lastRxMs) < 2000;
    if (hmFresh) {
        // RSSI label + value
        display->drawStr(4, 19, "RSSI");
        char tmp[12]; snprintf(tmp, sizeof(tmp), "%ddBm", (int)telem.rssi);
        display->drawStr(4, 25, tmp);
        // LQ bar
        display->drawStr(4, 31, "LQ");
        display->drawFrame(14, 26, 34, 5);
        uint8_t lqFill = (uint8_t)(telem.lq / 100.f * 32);
        if (lqFill > 0) display->drawBox(15, 27, lqFill, 3);
        // RX voltage
        display->drawStr(4, 38, "RXV");
        if (telem.rxVoltage > 0) {
            snprintf(tmp, sizeof(tmp), "%.1fV", telem.rxVoltage/1000.f);
        } else {
            snprintf(tmp, sizeof(tmp), "--");
        }
        display->drawStr(4, 44, tmp);
    } else {
        // RX offline message
        display->drawStr(4, 24, "RX");
        display->drawStr(4, 31, "off-");
        display->drawStr(4, 38, "line");
    }
    // Pot bars — one row each, label + bar
    display->setFont(u8g2_font_4x6_tr);
    display->drawStr(10, 55, "9");
    {
        float pct = constrain((float)(channels[8]-CH_MIN)/(CH_MAX-CH_MIN), 0.f, 1.f);
        display->drawFrame(16, 50, 38, 5);
        for (uint8_t xx=17;xx<53;xx+=2) display->drawPixel(xx, 52);
        uint8_t pos=(uint8_t)(pct*34)+1;
        display->drawBox(16+pos, 51, 2, 3);
    }
    display->drawStr(69, 55, "10");
    {
        float pct = constrain((float)(channels[9]-CH_MIN)/(CH_MAX-CH_MIN), 0.f, 1.f);
        display->drawFrame(78, 50, 38, 5);
        for (uint8_t xx=79;xx<117;xx+=2) display->drawPixel(xx, 52);
        uint8_t pos=(uint8_t)(pct*34)+1;
        display->drawBox(78+pos, 51, 2, 3);
    }

    // ── MODEL ICON (x=52..75, y=13..36) ─────────────────────────────────────
    drawModelIcon(52, 13, models[activeModel].type);

    // ── SWITCH STATES (x=77..122, y=13..53) ─────────────────────────────────
    display->setFont(u8g2_font_4x6_tr);
    {
        char sw[8];
        // CH5: 3-pos → 1/2/3
        snprintf(sw, sizeof(sw), "5:%d", sw3pos(channels[4]));
        display->drawStr(100, 19, sw);
        // CH6: 3-pos → 1/2/3
        snprintf(sw, sizeof(sw), "6:%d", sw3pos(channels[5]));
        display->drawStr(100, 27, sw);
        // CH7: 2-pos → ON/off
        bool a = channels[6]>1700;
        display->drawStr(100, 35, a ? "7:ON" : "7:off");
        // CH8: 2-pos → ON/off
        bool r = channels[7]>1700;
        display->drawStr(100, 43, r ? "8:ON" : "8:off");
    }

    // ── WARNINGS (overlay, highest priority) ─────────────────────────────────
    if (battVoltage > 0.1f && battVoltage < battCritV) {
        display->drawBox(29, 42, 70, 7);
        display->setDrawColor(0);
        display->drawStr(35, 48, "LOW BATTERY!");
        display->setDrawColor(1);
    }
    {
        bool isCar = (models[activeModel].type == MODEL_CAR);
        bool thrBad = isCar
            ? (channels[6] > 1700 && abs((int)channels[2] - (int)CH_MID) > 150)
            : (channels[6] > 1700 && (int)channels[2] > (int)(CH_THROTTLE_MIN + 150));
        if (thrBad) {
            display->drawBox(29, 42, 70, 7);
            display->setDrawColor(0);
            display->drawStr(35, 48, isCar ? "THR NOT CTR!" : "THR HIGH!");
            display->setDrawColor(1);
        }
    }

    display->sendBuffer();
}

void drawMenu() {
    display->clearBuffer();
    drawBreadcrumb("Menu");
    display->setFont(u8g2_font_6x10_tr);
    for (uint8_t i = 0; i < MENU_VISIBLE; i++) {
        uint8_t idx = menuOffset + i;
        if (idx >= MENU_COUNT) break;
        uint8_t y = 22 + i*12;
        if (idx == (uint8_t)menuCursor) {
            display->drawBox(0,y-9,128,11);
            display->setDrawColor(0);
            display->drawStr(4,y,menuItems[idx]);
            display->setDrawColor(1);
        } else {
            display->drawStr(4,y,menuItems[idx]);
        }
    }
    // Scroll indicators
    display->setFont(u8g2_font_4x6_tr);
    if (menuOffset > 0)               display->drawStr(122,17,"\x1e");
    if (menuOffset+MENU_VISIBLE < MENU_COUNT) display->drawStr(122,63,"\x1f");
    display->drawStr(2,63,"\x1e\x1f move  OK open");
    display->sendBuffer();
}

void drawModels() {
    display->clearBuffer();
    drawBreadcrumb("Models");
    display->setFont(u8g2_font_5x7_tr);
    for (uint8_t i = 0; i < MODEL_COUNT; i++) {
        uint8_t y = 20 + i*12;
        bool sel     = (i == (uint8_t)modelCursor);
        bool active  = (i == activeModel);
        if (sel) {
            display->drawBox(0,y-8,128,10);
            display->setDrawColor(0);
        }
        // Active model marker
        display->drawStr(2, y, active ? ">" : " ");
        display->drawStr(10, y, models[i].name);
        display->drawStr(80, y, modelTypeNames[models[i].type]);
        display->setDrawColor(1);
    }
    display->setFont(u8g2_font_4x6_tr);
    display->setFont(u8g2_font_4x6_tr);
    display->drawStr(2,63,"OK=use  Rt=edit  Up+OK=copy  Dn+OK=del");
    display->sendBuffer();
}

void drawChannels() {
    display->clearBuffer();
    drawBreadcrumb("Channels");
    // 10 channels in 5 rows x 2 columns, 10px row height
    // Col layout: LABEL(12px) BAR(30px) VALUE(20px) = 62px per col
    const char* lbl[] = {
        "AIL","ELE","THR","RUD",
        "FM ","CAM","ARM","RTH",
        "PA ","PB "
    };
    display->setFont(u8g2_font_4x6_tr);
    for (uint8_t i = 0; i < 10; i++) {
        uint8_t col = i % 2;
        uint8_t row = i / 2;
        uint8_t x   = col * 64;
        uint8_t y   = 13 + row * 10;
        // channel index: 0-7 map to channels[0-7], 8-9 map to channels[8-9] (pots)
        float pct = (float)(channels[i] - CH_MIN) / (CH_MAX - CH_MIN);
        display->drawStr(x + 1, y + 6, lbl[i]);
        drawBar(x + 14, y + 1, 30, 7, pct);
        char v[6]; snprintf(v, sizeof(v), "%4d", channels[i]);
        display->drawStr(x + 46, y + 6, v);
    }
    display->sendBuffer();
}

void drawAxisReverse() {
    display->clearBuffer();
    drawBreadcrumb("Axis Reverse");
    display->setFont(u8g2_font_5x7_tr);
    const char* axNames[] = {"Aileron","Elevator","Throttle","Rudder","Pot A","Pot B"};
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t y = 22 + i * 9;
        bool rev = models[activeModel].axCal[i].reversed;
        bool sel = (i == (uint8_t)settingCursor);
        if (sel) {
            display->drawBox(0, y-7, 128, 9);
            display->setDrawColor(0);
        }
        display->drawStr(2, y, axNames[i]);
        display->drawStr(90, y, rev ? "[REV]" : "[NRM]");
        display->setDrawColor(1);
    }
    display->setFont(u8g2_font_4x6_tr);
    display->drawStr(2, 63, "OK toggle  Back save");
    display->sendBuffer();
}

void drawEspNowPair() {
    display->clearBuffer();
    drawBreadcrumb("ESP-NOW Pair");
    display->setFont(u8g2_font_5x7_tr);
    switch (pairState) {
        case PAIR_IDLE:
            display->drawStr(2, 24, "Put receiver into");
            display->drawStr(2, 34, "pairing mode first.");
            display->drawStr(2, 46, "Then press OK.");
            break;
        case PAIR_SEARCHING: {
            display->drawStr(2, 22, "Searching...");
            // Animated dots
            uint8_t dots = (millis() / 400) % 4;
            char d[5] = "    "; for (uint8_t i=0;i<dots;i++) d[i]='.';
            display->drawStr(76, 22, d);
            // Countdown bar
            uint32_t elapsed = millis() - pairStartMs;
            float pct = 1.f - (float)elapsed / PAIR_TIMEOUT_MS;
            drawBar(2, 28, 124, 7, pct);
            display->drawStr(2, 46, "TX MAC:");
            uint8_t m[6]; esp_read_mac(m, ESP_MAC_WIFI_STA);
            char mac[20]; snprintf(mac,sizeof(mac),"%02X%02X%02X%02X%02X%02X",m[0],m[1],m[2],m[3],m[4],m[5]);
            display->setFont(u8g2_font_4x6_tr);
            display->drawStr(2, 55, mac);
            display->setFont(u8g2_font_5x7_tr);
            break;
        }
        case PAIR_SUCCESS: {
            display->drawStr(2, 22, "Paired!");
            char mac[20];
            snprintf(mac,sizeof(mac),"%02X%02X%02X%02X%02X%02X",
                pairedMac[0],pairedMac[1],pairedMac[2],pairedMac[3],pairedMac[4],pairedMac[5]);
            display->setFont(u8g2_font_4x6_tr);
            display->drawStr(2, 32, "RX MAC:");
            display->drawStr(2, 40, mac);
            display->setFont(u8g2_font_5x7_tr);
            display->drawStr(2, 52, "Saved to model.");
            display->drawStr(2, 62, "OK = back to Radio");
            break;
        }
        case PAIR_TIMEOUT:
            display->drawStr(2, 28, "No receiver found.");
            display->drawStr(2, 40, "Check RX is in");
            display->drawStr(2, 50, "pairing mode.");
            display->drawStr(2, 62, "OK = try again");
            break;
    }
    display->sendBuffer();
}

void drawRates() {
    display->clearBuffer();
    drawBreadcrumb("Expo & Rates");
    ModelProfile& m = models[activeModel];
    const char* axNames[] = {"AIL","ELE","THR","RUD"};

    // ── LEFT PANEL: channel list (x=0..57) ───────────────────────────────────
    display->setFont(u8g2_font_4x6_tr);
    display->drawStr(2, 18, "CH  RATE EXPO");
    display->drawHLine(0, 20, 58);

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t y   = 27 + i * 9;
        bool    sel = (i == (uint8_t)settingCursor);
        if (sel && !settingEditMode) {
            display->drawBox(0, y-6, 57, 8);
            display->setDrawColor(0);
        } else if (sel && settingEditMode) {
            // Edit mode — dashed border
            display->drawFrame(0, y-6, 57, 8);
        }
        char buf[20];
        snprintf(buf, sizeof(buf), "%-3s %3d%% %3d%%",
                 axNames[i], m.rate[i], m.expo[i]);
        display->drawStr(2, y, buf);
        display->setDrawColor(1);
    }

    // Freq row
    uint8_t fy = 27 + 4 * 9;
    bool freqSel = (settingCursor == 4);
    if (freqSel && !settingEditMode) {
        display->drawBox(0, fy-6, 57, 8);
        display->setDrawColor(0);
    } else if (freqSel && settingEditMode) {
        display->drawFrame(0, fy-6, 57, 8);
    }
    char fbuf[16];
    snprintf(fbuf, sizeof(fbuf), "TX  %3dHz", m.txFreqHz);
    display->drawStr(2, fy, fbuf);
    display->setDrawColor(1);

    // Hint at bottom left
    display->setFont(u8g2_font_4x6_tr);
    if (settingEditMode && settingCursor < 4) {
        display->drawStr(0, 63, "=expo Rt=rate");
    } else if (settingEditMode) {
        display->drawStr(0, 63, "/Rt=freq");
    } else {
        display->drawStr(0, 63, "OK=edit Bk=save");
    }

    // ── DIVIDER ───────────────────────────────────────────────────────────────
    display->drawVLine(59, 11, 53);

    // ── RIGHT PANEL: expo curve graph (x=61..127, y=11..63) ──────────────────
    // Graph area: 66px wide x 52px tall
    // Origin at centre: gx=94, gy=37
    const uint8_t GX  = 61;   // graph left edge
    const uint8_t GY  = 11;   // graph top edge
    const uint8_t GW  = 67;   // graph width
    const uint8_t GH  = 52;   // graph height
    const uint8_t CX  = GX + GW/2; // centre x = 94
    const uint8_t CY  = GY + GH/2; // centre y = 37
    const uint8_t R   = GH/2 - 1;  // plot radius = 25px

    // Draw axes
    display->drawHLine(GX, CY, GW);      // X axis
    display->drawVLine(CX, GY, GH);      // Y axis
    // Axis labels
    display->setFont(u8g2_font_4x6_tr);
    display->drawStr(GX+1, GY+5, "IN");
    display->drawStr(GX+1, GY+GH-1, "OUT");

    // Draw curve for selected axis (or freq row → show no curve)
    if (settingCursor < 4) {
        uint8_t  axIdx  = settingCursor;
        uint8_t  expo   = m.expo[axIdx];
        uint8_t  rate   = m.rate[axIdx];
        float    eNorm  = expo  / 100.0f;
        float    rNorm  = rate  / 100.0f;

        uint8_t prevPx = 0, prevPy = 0;
        for (int8_t step = -25; step <= 25; step++) {
            float x = step / 25.0f;                          // -1..+1
            float y = x*(1.f-eNorm) + x*x*x*eNorm;          // expo curve
            y *= rNorm;                                       // apply rate
            uint8_t px = (uint8_t)(CX + step);
            uint8_t py = (uint8_t)(CY - (int8_t)(y * R));
            py = (uint8_t)constrain((int)py, GY+1, GY+GH-2);
            if (step > -25) {
                // Draw line segment between points
                // Simple bresenham for small distances
                int dx = (int)px - (int)prevPx;
                int dy = (int)py - (int)prevPy;
                int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
                if (steps == 0) steps = 1;
                for (int s = 0; s <= steps; s++) {
                    uint8_t lx = prevPx + dx*s/steps;
                    uint8_t ly = prevPy + dy*s/steps;
                    display->drawPixel(lx, ly);
                }
            }
            prevPx = px; prevPy = py;
        }

        // Rate limit lines (dashed) — show max output
        uint8_t rLimit = (uint8_t)(R * rNorm);
        for (uint8_t xx = GX+2; xx < GX+GW-1; xx += 3) {
            display->drawPixel(xx, CY - rLimit);
            display->drawPixel(xx, CY + rLimit);
        }

        // Axis name + values top-right of graph
        char info[16];
        snprintf(info, sizeof(info), "%s", axNames[axIdx]);
        display->drawStr(GX+GW-display->getStrWidth(info)-1, GY+5, info);
        snprintf(info, sizeof(info), "R%d E%d", rate, expo);
        display->drawStr(GX+GW-display->getStrWidth(info)-1, GY+11, info);
    } else {
        // Freq selected — show text instead of curve
        display->drawStr(CX-12, CY-3, "FREQ");
        char fhz[8]; snprintf(fhz, sizeof(fhz), "%dHz", m.txFreqHz);
        uint8_t fw = display->getStrWidth(fhz);
        display->drawStr(CX - fw/2, CY+6, fhz);
    }

    display->sendBuffer();
}

void drawRadio() {
    display->clearBuffer();
    drawBreadcrumb("Radio");
    display->setFont(u8g2_font_6x10_tr);
    display->drawStr(2,23,"Mode:");
    if (radioMode==RADIO_NRF24) {
        display->drawBox(40,13,45,12); display->setDrawColor(0);
        display->drawStr(42,23,"nRF24"); display->setDrawColor(1);
        display->drawStr(90,23,"ESPNOW");
    } else {
        display->drawStr(40,23,"nRF24");
        display->drawBox(86,13,55,12); display->setDrawColor(0);
        display->drawStr(88,23,"ESPNOW"); display->setDrawColor(1);
    }
    display->setFont(u8g2_font_5x7_tr);
    display->drawStr(2,38,"Link:");
    drawBar(36,30,70,8,linkQuality/100.f);
    char lq[8]; snprintf(lq,sizeof(lq),"%d%%",linkQuality);
    display->drawStr(110,38,lq);
    display->drawStr(2,50,radioLinked?"Status: LINKED":"Status: no link");
    display->setFont(u8g2_font_4x6_tr);
    if (simModeOn) {
        display->drawStr(2,58, usbReady ? "USB: CONNECTED" : "USB: waiting for PC...");
    }
    display->setFont(u8g2_font_4x6_tr);
    char s[40]; snprintf(s,sizeof(s),"Rt=mode  OK=sim  Up=pair");
    display->drawStr(2,63,s);
    display->sendBuffer();
}

void drawCalibrate() {
    display->clearBuffer();
    drawBreadcrumb("Calibrate");
    display->setFont(u8g2_font_5x7_tr);
    switch (calibState) {
        case CALIB_IDLE:
            display->drawStr(2,24,"Calibrate gimbals");
            display->drawStr(2,36,"and potentiometers.");
            display->drawStr(2,50,"Press OK to start.");
            break;
        case CALIB_CENTER:
            display->drawStr(2,24,"1. Move all sticks");
            display->drawStr(2,34,"   to CENTER.");
            display->drawStr(2,44,"   Release pots.");
            display->drawStr(2,58,"OK = confirm center");
            break;
        case CALIB_EXTREME: {
            uint32_t elapsed = millis() - calibStart;
            uint32_t remaining = (elapsed < CALIB_EXTREME_MS)
                               ? (CALIB_EXTREME_MS - elapsed) / 1000 + 1 : 0;
            display->drawStr(2,24,"2. Move sticks to");
            display->drawStr(2,34,"   ALL extremes.");
            char t[24]; snprintf(t,sizeof(t),"   Time: %lus", (unsigned long)remaining);
            display->drawStr(2,44,t);
            display->drawStr(2,58,"OK = done");
            break;
        }
        case CALIB_DONE:
            display->drawStr(2,28,"Calibration saved!");
            display->drawStr(2,42,"OK = back to menu");
            break;
    }
    display->sendBuffer();
}

void drawSettings() {
    display->clearBuffer();
    display->setFont(u8g2_font_5x7_tr);
    if (settingEditMode) {
        display->drawStr(2,8,"\x7f "); display->drawStr(10,8,"Settings [EDIT]");
    } else {
        drawBreadcrumb("Settings");
    }
    for (uint8_t i=0;i<SETTINGS_COUNT;i++) {
        uint8_t y=22+i*18;
        bool sel=(i==(uint8_t)settingCursor);
        if (sel) {
            if (settingEditMode) {
                display->drawFrame(0,y-8,128,10);
                display->drawStr(1,y,"\x1e"); display->drawStr(119,y,"\x1f");
            } else {
                display->drawBox(0,y-8,128,10); display->setDrawColor(0);
            }
        }
        display->drawStr(12,y,settings[i].label);
        char v[8];
        if (settings[i].divisor > 1)
            snprintf(v,sizeof(v),"%.1f",settings[i].value / (float)settings[i].divisor);
        else
            snprintf(v,sizeof(v),"%d",settings[i].value);
        display->drawStr(70,y,v);
        display->setDrawColor(1);
        float pct=(float)(settings[i].value-settings[i].minVal)/
                  (float)(settings[i].maxVal-settings[i].minVal);
        drawBar(90,y-7,34,8,pct);
    }
    display->setFont(u8g2_font_4x6_tr);
    display->drawStr(2,63,settingEditMode
        ? "\x1e\x1f adjust  OK/Back confirm"
        : "\x1e\x1f select  OK edit  Back menu");
    display->sendBuffer();
}

void drawTelemetry() {
    display->clearBuffer();
    drawBreadcrumb("Telemetry");
    display->setFont(u8g2_font_5x7_tr);

    bool fresh = telem.fresh && (millis() - telem.lastRxMs) < 2000;

    if (!fresh) {
        display->drawStr(2, 28, "No telemetry data.");
        display->drawStr(2, 40, "Connect RX via ESP-NOW.");
        display->setFont(u8g2_font_4x6_tr);
        display->drawStr(2, 62, "Back = menu");
        display->sendBuffer();
        return;
    }

    char buf[32];
    // RSSI
    snprintf(buf, sizeof(buf), "RSSI: %ddBm", (int)telem.rssi);
    display->drawStr(2, 22, buf);
    // Signal quality bar
    float rssiPct = constrain(((float)telem.rssi + 100.f) / 60.f, 0.f, 1.f);
    drawBar(70, 15, 55, 8, rssiPct);

    // LQ
    snprintf(buf, sizeof(buf), "LQ:   %d%%", (int)telem.lq);
    display->drawStr(2, 33, buf);
    drawBar(70, 26, 55, 8, telem.lq / 100.f);

    // RX voltage
    if (telem.rxVoltage > 0) {
        snprintf(buf, sizeof(buf), "RXV:  %.2fV", telem.rxVoltage / 1000.f);
    } else {
        snprintf(buf, sizeof(buf), "RXV:  --");
    }
    display->drawStr(2, 44, buf);

    // ARM state
    snprintf(buf, sizeof(buf), "ARM:  %s", telem.rxArmed ? "ARMED" : "disarmed");
    display->drawStr(2, 55, buf);

    // Age
    uint32_t ageSec = (millis() - telem.lastRxMs) / 1000;
    snprintf(buf, sizeof(buf), "Age: %lus", (unsigned long)ageSec);
    display->setFont(u8g2_font_4x6_tr);
    display->drawStr(90, 63, buf);
    display->drawStr(2, 63, "Back = menu");

    display->sendBuffer();
}

void drawAbout() {
    display->clearBuffer();
    drawBreadcrumb("About");
    display->setFont(u8g2_font_5x7_tr);
    display->drawStr(0,22,"RC Transmitter v3.1");
    char l[28];
    snprintf(l,sizeof(l),"Chip:  %s",ESP.getChipModel());     display->drawStr(0,32,l);
    snprintf(l,sizeof(l),"Flash: %dMB",(int)(ESP.getFlashChipSize()>>20)); display->drawStr(0,42,l);
    snprintf(l,sizeof(l),"PSRAM: %dKB",(int)(ESP.getPsramSize()/1024)); display->drawStr(0,52,l);
    snprintf(l,sizeof(l),"Heap:  %luKB",(unsigned long)(ESP.getFreeHeap()/1024)); display->drawStr(0,62,l);
    display->sendBuffer();
}

void drawModelEdit() {
    display->clearBuffer();
    drawBreadcrumb("Edit Model");
    display->setFont(u8g2_font_5x7_tr);

    ModelProfile& m = models[editModelIdx];

    // ── Name field ────────────────────────────────────────────────────────────
    bool nameSel = (editField == 0);

    if (nameSel && !editFieldMode) {
        display->drawBox(0, 13, 128, 11);
        display->setDrawColor(0);      // Black text on white background
    } else {
        display->setDrawColor(1);      // White text on black background
    }

    display->drawStr(2, 22, "Name:");

    if (nameSel && editFieldMode) {
        display->setDrawColor(1);

        char nameBuf[17];
        strncpy(nameBuf, m.name, 16);
        nameBuf[16] = 0;

        uint8_t nx = 38;

        for (uint8_t i = 0; i < 10 && nameBuf[i]; i++) {
            char ch[2] = { nameBuf[i], 0 };

            if (i == nameCursor) {
                // Highlight current character
                display->drawBox(nx - 1, 14, 7, 9);

                display->setDrawColor(0);      // Black character on white box
                display->drawStr(nx, 22, ch);
                display->setDrawColor(1);
            } else {
                display->drawStr(nx, 22, ch);
            }

            nx += 7;
        }

        // Cursor after last character
        uint8_t len = strlen(nameBuf);
        if (nameCursor >= len && nameCursor < 16) {
            uint8_t cx = 38 + nameCursor * 7;

            display->drawBox(cx - 1, 14, 7, 9);

            display->setDrawColor(0);
            display->drawStr(cx, 22, "_");    // or " "
            display->setDrawColor(1);
        }
    } else {
        display->drawStr(38, 22, m.name);
    }

    display->setDrawColor(1);

    // ── Type field ────────────────────────────────────────────────────────────
    bool typeSel = (editField == 1);

    if (typeSel && !editFieldMode) {
        display->drawBox(0, 26, 128, 11);
        display->setDrawColor(0);
    } else {
        display->setDrawColor(1);
    }

    display->drawStr(2, 35, "Type:");

    if (typeSel && editFieldMode) {
        display->drawStr(36, 35, "\x7F ");
        display->drawStr(44, 35, modelTypeNames[m.type]);
        uint8_t tw = display->getStrWidth(modelTypeNames[m.type]);
        display->drawStr(46 + tw, 35, " x");
    } else {
        display->drawStr(38, 35, modelTypeNames[m.type]);
    }

    display->setDrawColor(1);

    // ── Mixer field ───────────────────────────────────────────────────────────
    bool mixerSel = (editField == 2);

    if (mixerSel && !editFieldMode) {
        display->drawBox(0, 39, 128, 11);
        display->setDrawColor(0);
    } else {
        display->setDrawColor(1);
    }

    display->drawStr(2, 48, "Mixer:");

    if (mixerSel && editFieldMode) {
        display->drawStr(40, 48, "\x7F ");
        display->drawStr(48, 48, mixerTypeNames[m.mixerType]);
        uint8_t mw = display->getStrWidth(mixerTypeNames[m.mixerType]);
        display->drawStr(50 + mw, 48, " x");
    } else {
        display->drawStr(42, 48, mixerTypeNames[m.mixerType]);
    }

    display->setDrawColor(1);

    // ── Hint ──────────────────────────────────────────────────────────────────
    display->setFont(u8g2_font_4x6_tr);

    if (editFieldMode && editField == 0) {
        display->drawStr(2, 63, "^v char  -> next  OK done");
    } else if (editFieldMode && (editField == 1 || editField == 2)) {
        display->drawStr(2, 63, "^v change   OK/<- done");
    } else {
        display->drawStr(2, 63, "^v select  OK edit  <- save");
    }

    display->sendBuffer();
}

void drawChMap() {
    display->clearBuffer();
    drawBreadcrumb("Ch Map");
    display->setFont(u8g2_font_4x6_tr);

    ModelProfile& m = models[activeModel];

    // Two-column layout: CH1-CH5 left, CH6-CH10 right
    display->drawStr(2,  17, "OUT SRC");
    display->drawStr(66, 17, "OUT SRC");
    display->drawHLine(0, 19, 128);

    for (uint8_t i = 0; i < 10; i++) {
        uint8_t col = (i >= 5) ? 1 : 0;
        uint8_t row = i % 5;
        uint8_t x   = col * 64;
        uint8_t y   = 26 + row * 7;
        bool    sel = (i == (uint8_t)settingCursor);

        if (sel) {
            if (settingEditMode) {
                display->drawFrame(x, y-5, 62, 7); // dashed border = edit mode
            } else {
                display->drawBox(x, y-5, 62, 7);
                display->setDrawColor(0);
            }
        }
        char buf[10];
        snprintf(buf, sizeof(buf), "C%-2d C%-2d", i+1, m.chRemap[i]+1);
        display->drawStr(x+2, y, buf);
        display->setDrawColor(1);
    }

    display->drawStr(2, 63, settingEditMode
        ? "\x1e\x1f src  OK/Bk confirm"
        : "\x1e\x1f sel  OK edit  Bk save");
    display->sendBuffer();
}

void renderFrame() {
    switch (uiState) {
        case UI_HOME:      drawHome();      break;
        case UI_MENU:      drawMenu();      break;
        case UI_MODELS:    drawModels();    break;
        case UI_MODEL_EDIT: drawModelEdit(); break;
        case UI_CHANNELS:     drawChannels();    break;
        case UI_CH_MAP:       drawChMap();       break;
        case UI_AXIS_REVERSE: drawAxisReverse(); break;
        case UI_RATES:        drawRates();       break;
        case UI_RADIO:        drawRadio();       break;
        case UI_ESPNOW_PAIR:  drawEspNowPair();  break;
        case UI_CALIBRATE: drawCalibrate(); break;
        case UI_SETTINGS:  drawSettings();  break;
        case UI_TELEMETRY: drawTelemetry(); break;
        case UI_ABOUT:     drawAbout();     break;
    }
}

// =============================================================================
// INPUT HANDLER
// =============================================================================
void handleInput(BtnEvent evt) {
    switch (uiState) {

        case UI_HOME:
            if (evt==BTN_OK) { uiState=UI_MENU; menuCursor=0; menuOffset=0; }
            break;

        case UI_MENU:
            if (evt==BTN_UP) {
                if (menuCursor>0) menuCursor--;
                if (menuCursor < menuOffset) menuOffset = menuCursor;
            } else if (evt==BTN_DOWN) {
                if (menuCursor<MENU_COUNT-1) menuCursor++;
                if (menuCursor >= menuOffset+MENU_VISIBLE) menuOffset=menuCursor-MENU_VISIBLE+1;
            } else if (evt==BTN_BACK) {
                uiState=UI_HOME;
            } else if (evt==BTN_OK) {
                switch (menuCursor) {
                    case 0:  uiState=UI_HOME;                                                        break;
                    case 1:  uiState=UI_MODELS;       modelCursor=activeModel;                     break;
                    case 2:  uiState=UI_CHANNELS;                                                   break;
                    case 3:  uiState=UI_CH_MAP;        settingCursor=0; settingEditMode=false;      break;
                    case 4:  uiState=UI_AXIS_REVERSE;  settingCursor=0;                             break;
                    case 5:  uiState=UI_RATES;         settingCursor=0; settingEditMode=false;      break;
                    case 6:  uiState=UI_RADIO;                                                      break;
                    case 7:  uiState=UI_TELEMETRY;                                                  break;
                    case 8:  uiState=UI_CALIBRATE;     calibState=CALIB_IDLE;                      break;
                    case 9:  uiState=UI_SETTINGS;      settingCursor=0; settingEditMode=false;      break;
                    case 10: uiState=UI_ABOUT;                                                      break;
                }
            }
            break;

        case UI_MODELS: {
            static bool upHeld = false, downHeld = false;
            if (evt==BTN_UP)   { modelCursor=(modelCursor+MODEL_COUNT-1)%MODEL_COUNT; upHeld=true; downHeld=false; }
            if (evt==BTN_DOWN) { modelCursor=(modelCursor+1)%MODEL_COUNT; downHeld=true; upHeld=false; }
            if (evt==BTN_BACK) { uiState=UI_MENU; upHeld=downHeld=false; }
            if (evt==BTN_OK) {
                if (upHeld) {
                    // Copy: duplicate active model into this slot
                    uint8_t src = activeModel;
                    models[modelCursor] = models[src];
                    snprintf(models[modelCursor].name, 16, "Copy %d", modelCursor+1);
                    modelsSave(); beep(2,40,40);
                    upHeld=false;
                } else if (downHeld) {
                    // Remove: reset slot to default
                    snprintf(models[modelCursor].name, 16, "Model %d", modelCursor+1);
                    models[modelCursor].type      = MODEL_AIRPLANE;
                    models[modelCursor].radioMode = RADIO_NRF24;
                    for (int i=0;i<6;i++) models[modelCursor].axCal[i]={0,ADC_MAX/2,ADC_MAX,false};
                    models[modelCursor].axCal[1].reversed=true;
                    for (int i=0;i<4;i++){models[modelCursor].expo[i]=0;models[modelCursor].rate[i]=100;}
                    models[modelCursor].txFreqHz=50;
                    models[modelCursor].mixerType=MIXER_NONE;
                    for (int i=0;i<10;i++) models[modelCursor].chRemap[i]=i;
                    if (activeModel==modelCursor) activeModel=0;
                    modelsSave(); beep(3,30,30);
                    downHeld=false;
                } else {
                    // Activate
                    activeModel=modelCursor;
                    modelApply(); modelsSave();
                    beep(1,60,0); uiState=UI_MENU;
                }
            }
            if (evt==BTN_RIGHT) {
                editModelIdx=modelCursor; editField=0;
                editFieldMode=false; nameCursor=0;
                uiState=UI_MODEL_EDIT; upHeld=downHeld=false;
            }
            break;
        }

        case UI_MODEL_EDIT: {
            ModelProfile& em = models[editModelIdx];
            const uint8_t EDIT_FIELD_COUNT = 3; // Name, Type, Mixer
            if (!editFieldMode) {
                // Field selection mode
                if (evt==BTN_UP)   editField = (editField + EDIT_FIELD_COUNT - 1) % EDIT_FIELD_COUNT;
                if (evt==BTN_DOWN) editField = (editField + 1) % EDIT_FIELD_COUNT;
                if (evt==BTN_OK) {
                    editFieldMode = true;
                    if (editField == 0) nameCursor = strlen(em.name);
                    beep(1,30,0);
                }
                if (evt==BTN_BACK) {
                    // Save only the edited slot, not all 4 — cheaper NVS write
                    modelSaveSlot(editModelIdx);
                    beep(1,60,0);
                    uiState = UI_MODELS;
                }
            } else {
                // Editing a field
                if (editField == 0) {
                    // Name editing
                    uint8_t len = strlen(em.name);
                    if (evt==BTN_UP) {
                        char cur = (nameCursor < len) ? em.name[nameCursor] : ' ';
                        uint8_t idx = (charToIdx(cur) + 1) % CHARSET_LEN;
                        if (nameCursor >= len) { em.name[nameCursor] = CHARSET[idx]; em.name[nameCursor+1]=0; }
                        else em.name[nameCursor] = CHARSET[idx];
                    } else if (evt==BTN_DOWN) {
                        char cur = (nameCursor < len) ? em.name[nameCursor] : ' ';
                        uint8_t idx = (charToIdx(cur) + CHARSET_LEN - 1) % CHARSET_LEN;
                        if (nameCursor >= len) { em.name[nameCursor] = CHARSET[idx]; em.name[nameCursor+1]=0; }
                        else em.name[nameCursor] = CHARSET[idx];
                    } else if (evt==BTN_RIGHT) {
                        // Advance cursor
                        if (nameCursor < 14) nameCursor++;
                    } else if (evt==BTN_BACK) {
                        if (nameCursor > 0) nameCursor--;
                        else { editFieldMode = false; }
                    } else if (evt==BTN_OK) {
                        editFieldMode = false;
                        beep(1,30,0);
                    }
                } else if (editField == 1) {
                    // Type editing
                    if (evt==BTN_UP || evt==BTN_RIGHT)
                        em.type = (ModelType)((em.type + 1) % 4);
                    if (evt==BTN_DOWN)
                        em.type = (ModelType)((em.type + 2) % 4);
                    if (evt==BTN_OK || evt==BTN_BACK) {
                        editFieldMode = false;
                        beep(1,30,0);
                    }
                } else {
                    // Mixer editing — cycles None/Elevon/V-Tail/Flaperon
                    if (evt==BTN_UP || evt==BTN_RIGHT)
                        em.mixerType = (MixerType)((em.mixerType + 1) % 4);
                    if (evt==BTN_DOWN)
                        em.mixerType = (MixerType)((em.mixerType + 3) % 4);
                    if (evt==BTN_OK || evt==BTN_BACK) {
                        editFieldMode = false;
                        beep(1,30,0);
                    }
                }
            }
            break;
        }

        case UI_CHANNELS:
            if (evt==BTN_BACK || evt==BTN_OK) uiState=UI_MENU;
            break;

        case UI_AXIS_REVERSE:
            if (evt==BTN_UP)   settingCursor = (settingCursor + 5) % 6;
            if (evt==BTN_DOWN) settingCursor = (settingCursor + 1) % 6;
            if (evt==BTN_OK) {
                models[activeModel].axCal[settingCursor].reversed =
                    !models[activeModel].axCal[settingCursor].reversed;
                // Only mark dirty here — actual NVS write happens once on exit (Back)
                axisReverseDirty = true;
                beep(1,30,0);
            }
            if (evt==BTN_BACK) {
                if (axisReverseDirty) { modelSaveActive(); axisReverseDirty = false; }
                uiState=UI_MENU;
            }
            break;

        case UI_RATES: {
            ModelProfile& mr = models[activeModel];
            const uint8_t RATES_ROWS = 5; // 4 axes + freq
            if (!settingEditMode) {
                if (evt==BTN_UP)   settingCursor=(settingCursor+RATES_ROWS-1)%RATES_ROWS;
                if (evt==BTN_DOWN) settingCursor=(settingCursor+1)%RATES_ROWS;
                if (evt==BTN_OK)   { settingEditMode=true; beep(1,30,0); }
                if (evt==BTN_BACK) { modelSaveActive(); uiState=UI_MENU; }
            } else {
                if (settingCursor < 4) {
                    // Expo: Up/Down adjust, Right adjusts rate
                    if (evt==BTN_UP)
                        mr.expo[settingCursor] = constrain(mr.expo[settingCursor]+5, 0, 100);
                    if (evt==BTN_DOWN)
                        mr.expo[settingCursor] = constrain(mr.expo[settingCursor]-5, 0, 100);
                    if (evt==BTN_RIGHT)
                        mr.rate[settingCursor] = constrain(mr.rate[settingCursor]+5, 50, 100);
                    if (evt==BTN_BACK && mr.rate[settingCursor] > 50)
                        mr.rate[settingCursor] = constrain(mr.rate[settingCursor]-5, 50, 100);
                    else if (evt==BTN_BACK && mr.rate[settingCursor] == 50)
                        { settingEditMode=false; beep(1,30,0); }
                    if (evt==BTN_OK) { settingEditMode=false; beep(1,30,0); }
                } else {
                    // Freq: cycle through 25/50/100 Hz
                    static const uint8_t freqOptions[] = {25, 50, 100};
                    uint8_t fi = 1;
                    for (uint8_t k=0;k<3;k++) if (freqOptions[k]==mr.txFreqHz) fi=k;
                    if (evt==BTN_UP || evt==BTN_RIGHT)
                        mr.txFreqHz = freqOptions[(fi+1)%3];
                    if (evt==BTN_DOWN)
                        mr.txFreqHz = freqOptions[(fi+2)%3];
                    if (evt==BTN_OK || evt==BTN_BACK) { settingEditMode=false; beep(1,30,0); }
                }
            }
            break;
        }

        case UI_RADIO:
            if (evt==BTN_BACK) {
                uiState=UI_MENU;
            } else if (evt==BTN_RIGHT) {
                switchRadioMode(radioMode==RADIO_NRF24 ? RADIO_ESPNOW : RADIO_NRF24);
                saveSettings();
            } else if (evt==BTN_OK) {
                simModeOn=!simModeOn;
                beep(simModeOn?2:1,50,50);
            } else if (evt==BTN_UP) {
                // Enter ESP-NOW pairing mode
                if (radioMode != RADIO_ESPNOW) switchRadioMode(RADIO_ESPNOW);
                pairState = PAIR_IDLE;
                uiState   = UI_ESPNOW_PAIR;
            }
            break;

        case UI_ESPNOW_PAIR:
            if (evt==BTN_BACK) {
                pairState = PAIR_IDLE;
                uiState   = UI_RADIO;
            } else if (evt==BTN_OK) {
                if (pairState == PAIR_IDLE || pairState == PAIR_TIMEOUT) {
                    pairingStart();
                } else if (pairState == PAIR_SUCCESS) {
                    uiState = UI_RADIO;
                }
            }
            break;

        case UI_CALIBRATE:
            if (evt==BTN_BACK) {
                calibState=CALIB_IDLE;
                uiState=UI_MENU;
            } else if (evt==BTN_OK) {
                if (calibState==CALIB_IDLE) {
                    calibBegin();
                } else if (calibState==CALIB_CENTER || calibState==CALIB_EXTREME) {
                    calibConfirm();
                } else if (calibState==CALIB_DONE) {
                    calibState=CALIB_IDLE;
                    uiState=UI_MENU;
                }
            }
            break;

        case UI_SETTINGS:
            if (!settingEditMode) {
                if (evt==BTN_UP)   settingCursor=(settingCursor+SETTINGS_COUNT-1)%SETTINGS_COUNT;
                if (evt==BTN_DOWN) settingCursor=(settingCursor+1)%SETTINGS_COUNT;
                if (evt==BTN_OK)   { settingEditMode=true; beep(1,30,0); }
                if (evt==BTN_BACK) { uiState=UI_MENU; }
            } else {
                if (evt==BTN_UP) {
                    settings[settingCursor].value=constrain(
                        settings[settingCursor].value+settings[settingCursor].step,
                        settings[settingCursor].minVal,settings[settingCursor].maxVal);
                    applySetting(settingCursor);
                } else if (evt==BTN_DOWN) {
                    settings[settingCursor].value=constrain(
                        settings[settingCursor].value-settings[settingCursor].step,
                        settings[settingCursor].minVal,settings[settingCursor].maxVal);
                    applySetting(settingCursor);
                } else if (evt==BTN_OK || evt==BTN_BACK) {
                    settingEditMode=false; beep(1,30,0);
                }
            }
            break;

        case UI_CH_MAP: {
            ModelProfile& cm = models[activeModel];
            if (!settingEditMode) {
                if (evt==BTN_UP)   settingCursor = (settingCursor + 9) % 10;
                if (evt==BTN_DOWN) settingCursor = (settingCursor + 1) % 10;
                if (evt==BTN_OK)   { settingEditMode=true; beep(1,30,0); }
                if (evt==BTN_BACK) { modelSaveActive(); settingEditMode=false; uiState=UI_MENU; }
            } else {
                // Editing source for selected output slot
                if (evt==BTN_UP || evt==BTN_RIGHT)
                    cm.chRemap[settingCursor] = (cm.chRemap[settingCursor] + 1) % 10;
                if (evt==BTN_DOWN)
                    cm.chRemap[settingCursor] = (cm.chRemap[settingCursor] + 9) % 10;
                if (evt==BTN_OK || evt==BTN_BACK) { settingEditMode=false; beep(1,30,0); }
            }
            break;
        }

        case UI_TELEMETRY:
            if (evt==BTN_BACK || evt==BTN_OK) uiState=UI_MENU;
            break;

        case UI_ABOUT:
            if (evt==BTN_BACK || evt==BTN_OK) uiState=UI_MENU;
            break;
    }
}

// =============================================================================
// RADIO TASK — Core 0
// Handles all time-critical I/O: PCF8575 poll, ADC read, radio sends.
// Pinned to Core 0 alongside the WiFi/ESP-NOW stack.
// Priority 2 > loop()'s priority 1 — ensures radio timing is not delayed by
// display rendering or USB HID calls on Core 1.
// =============================================================================
void radioTask(void* /*pvParams*/) {
    uint32_t tInput = 0, tAdc = 0, tSend = 0;

    for (;;) {
        uint32_t now = millis();

        // ── PCF8575 + switch channels — 100 Hz ───────────────────────────────
        if (now - tInput >= 10) {
            tInput = now;
            pcfRead(); // I2C — no mutex (see sharedMux comment above)
            xSemaphoreTake(sharedMux, portMAX_DELAY);
            readSwitches();    // writes channels[4..7] from pcfCache
            flightTimerTick(); // reads channels[6]
            xSemaphoreGive(sharedMux);
        }

        // ── ADC gimbal/pot channels — 50 Hz ──────────────────────────────────
        if (now - tAdc >= 20) {
            tAdc = now;
            xSemaphoreTake(sharedMux, portMAX_DELAY);
            readAnalog();  // writes channels[0..3, 8, 9] + applies mixer
            calibStep();   // no-op unless calibrating — tracks min/max
            xSemaphoreGive(sharedMux);
        }

        // ── Radio send — at model txFreqHz (25/50/100 Hz) ────────────────────
        uint32_t radioInterval = 1000 / models[activeModel].txFreqHz;
        if (now - tSend >= radioInterval) {
            tSend = now;
            xSemaphoreTake(sharedMux, portMAX_DELAY);
            if (radioMode == RADIO_NRF24)  nrfSend();
            if (radioMode == RADIO_ESPNOW) espnowSend();
            if (simModeOn)                 simSend();
            xSemaphoreGive(sharedMux);
            pairingTick(); // outside mutex — calls esp_now_send + beep()
        }

        vTaskDelay(1); // yield — lets WiFi/ESP-NOW stack on Core 0 run
    }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500); // brief pause for Serial Monitor to attach

    Serial.println("\n=== RC Transmitter v3.1 ===");


    ledcAttach(PIN_LCD_BL, 5000, 8);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);


    // Load settings and models from NVS
    modelsDefault();
    modelsLoad();
    modelApply(); // load active model's saved ESP-NOW peer MAC before espnowInit() runs
    loadSettings();
    ledcWrite(PIN_LCD_BL, settings[1].value);
    radioMode = models[activeModel].radioMode;

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    Wire.setTimeout(50);
    // PCF8575 init via Mischianti library
    // Input pins (buttons + switches)
    for (int i = 0; i <= 10; i++) pcf8575.pinMode(i, INPUT);
    // Output pins (LED + buzzer, active LOW)
    pcf8575.pinMode(PCF_LED,    OUTPUT);
    pcf8575.pinMode(PCF_BUZZER, OUTPUT);
    pcf8575.pinMode(PCF_VIBRO,  OUTPUT);
    pcf8575.begin();
    // Outputs off initially (HIGH = inactive for active-LOW)
    pcfSetPin(PCF_LED,    false);
    pcfSetPin(PCF_BUZZER, false);
    pcfSetPin(PCF_VIBRO,  false);
    pcfRead(); // initial cache
    Serial.println("[INIT] PCF8575 OK");
    ledOn(); // LED on during rest of init

    display = new U8G2_ST7565_ERC12864_ALT_F_4W_SW_SPI(U8G2_R0, PIN_SPI_SCK, PIN_SPI_MOSI, PIN_LCD_CS, PIN_LCD_DC, U8X8_PIN_NONE);
    display->begin();
    display->setContrast(60);
    Serial.println("[INIT] Display OK");

    // Boot animation — expanding frame + title fade-in
    display->drawXBMP(0, 0, 128, 64, icon_logo);
    display->sendBuffer();
    delay(3000);

    ina219 = new Adafruit_INA219(INA219_ADDR);
    inaOk  = ina219->begin();
    if (inaOk) ina219->setCalibration_32V_1A();
    Serial.printf("[INIT] INA219: %s\n", inaOk?"OK":"not found");

   // nrfOk    = nrfInit();
    espnowOk = espnowInit();

    gamepad = new USBHIDGamepad();
    gamepad->begin();
    USB.productName("RC Transmitter v4");
    USB.manufacturerName("RC-TX");
    USB.begin();

    channelsInit();

    // Throttle-low startup safety check — refuse to leave the boot screen
    // normally if throttle stick is not at minimum. Prevents an accidental
    // high-throttle arm immediately after power-on.
    readAnalog();
    if (channels[2] > (CH_THROTTLE_MIN + 100)) {
        display->clearBuffer();
        display->setFont(u8g2_font_6x10_tr);
        display->drawStr(4, 24, "THROTTLE NOT");
        display->drawStr(4, 38, "AT MINIMUM!");
        display->setFont(u8g2_font_5x7_tr);
        display->drawStr(4, 54, "Lower stick to continue");
        display->sendBuffer();
        Serial.println("[SAFETY] Throttle high at boot — waiting for minimum");
        while (true) {
            readAnalog();
            if (channels[2] <= (CH_THROTTLE_MIN + 100)) break;
            beep(1, 100, 0);
            delay(400); // intentional — safety gate, not a hot path
        }
        Serial.println("[SAFETY] Throttle confirmed low — continuing boot");
    }

    beep(2); // non-blocking — continues via buzzTick() in loop
    ledOn();
    Serial.printf("[INIT] Ready — Model:%s  Radio:%s  Heap:%luKB\n",
        models[activeModel].name, radioModeName(),
        (unsigned long)(ESP.getFreeHeap()/1024));

    // Launch radio task on Core 0 — must be last, after all peripherals are ready
    sharedMux = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(
        radioTask,  // function
        "radio",    // task name
        4096,       // stack bytes (ADC + I2C + SPI + ESP-NOW headroom)
        nullptr,    // parameter
        2,          // priority (higher than loop's 1 — radio timing takes precedence)
        nullptr,    // handle (unused)
        0           // Core 0 (alongside WiFi/ESP-NOW stack)
    );
    Serial.println("[INIT] Radio task started on Core 0");
}

// =============================================================================
// LOOP
// =============================================================================
static uint32_t tBattery=0, tRender=0, tDebug=0;

void loop() {
    uint32_t now = millis();

    buzzTick(now);  // advance non-blocking beep state machine
    vibroTick(now); // advance non-blocking vibro state machine

    pollButtons();
    if (btnPending != BTN_NONE) {
        // Any button press wakes backlight
        if (dimmed) {
    ledcWrite(PIN_LCD_BL, settings[1].value);
            dimmed = false;
        }
        resetActivity();
        handleInput(btnPending);
        btnPending=BTN_NONE;
    }

    // PCF8575 poll, ADC reads, and radio sends are handled by radioTask on Core 0.
    // loop() on Core 1 handles UI, display, battery monitoring, and USB.

    if (now-tBattery >= 1100) {
        tBattery=now;
        bool safeToBatRead = (uiState != UI_CALIBRATE);
        if (inaOk && safeToBatRead) {
            float v=ina219->getBusVoltage_V()+(ina219->getShuntVoltage_mV()/1000.f);
            float c=ina219->getCurrent_mA()/1000.f;
            float p=ina219->getPower_mW()/1000.f;
            if (v>0.1f) {
                // EMA smoothing — avoids jumpy readings from instantaneous current draw spikes
                const float BATT_ALPHA = 0.2f;
                battVoltage = (battVoltage < 0.1f) ? v : (BATT_ALPHA*v + (1-BATT_ALPHA)*battVoltage);
                battCurrent = (battCurrent < 0.01f && c < 0.01f) ? c : (BATT_ALPHA*c + (1-BATT_ALPHA)*battCurrent);
                battPower   = p; // power is derived, no need to double-filter
            }
        }
        // LED blink on low battery (via PCF8575)
        static bool lowBattLedTog = false;
        if (battVoltage>0.1f && battVoltage<battWarnV) {
            lowBattLedTog = !lowBattLedTog;
            lowBattLedTog ? ledOn() : ledOff();
        } else {
            ledOn();
        }

        // Battery buzzer pattern
        if (battVoltage>0.1f && battVoltage<battCritV) {
            // Critical: fast triple beep every 5s
            if (now-lastBuzzMs>5000) { lastBuzzMs=now; beep(3,80,60); vibrate(3,150,100); }
        } else if (battVoltage>0.1f && battVoltage<battWarnV) {
            // Warning: single beep every 15s
            if (now-lastBuzzMs>15000) { lastBuzzMs=now; beep(1,120,0); vibrate(1,250,0); }
        }
    }

    // Backlight auto-dim
    if (!dimmed && (now-lastActivityMs) > DIM_TIMEOUT_MS) {
        ledcWrite(PIN_LCD_BL, DIM_LEVEL);
        dimmed = true;
    }

    // Throttle warning — beep once if ARM on with throttle up
    {
        static uint32_t lastThrottleWarnMs = 0;
        bool armed = channels[6] > 1700;
        bool throttleHigh = channels[2] > (CH_THROTTLE_MIN + 150);
        if (armed && throttleHigh && (now - lastThrottleWarnMs) > 3000) {
            lastThrottleWarnMs = now;
            beep(2, 150, 100);
            vibrate(2, 200, 150);
            Serial.println("[WARN] ARM on with throttle high!");
        }
    }

    // ── Connection lost alert ────────────────────────────────────────────────
    // For ESP-NOW: based on telemetry freshness (RX actively replying).
    // For nRF24:   based on radioLinked (ack success rate from nrfSend()).
    {
        bool isLinked;
        if (radioMode == RADIO_ESPNOW) {
            isLinked = telem.fresh && (now - telem.lastRxMs) < 2000;
        } else {
            isLinked = radioLinked;
        }

        if (wasLinked && !isLinked) {
            // Just lost connection — vibro only, no buzzer
            vibrate(4, 200, 120);
            lastConnLostMs = now;
            Serial.println("[WARN] Connection lost!");
        } else if (!isLinked && (now - lastConnLostMs) > 4000) {
            // Still lost — repeat alert every 4s, vibro only
            lastConnLostMs = now;
            vibrate(2, 250, 150);
        }
        wasLinked = isLinked;
    }

    // 30fps render — guarded by sharedMux so radioTask doesn't write channels[]
    // mid-frame. SW SPI sendBuffer takes ~1-2ms; radio task at 50Hz has 20ms
    // per cycle so the brief block is well within timing budget.
    if (now-tRender >= 33) {
        tRender=now;
        xSemaphoreTake(sharedMux, portMAX_DELAY);
        renderFrame();
        xSemaphoreGive(sharedMux);
        pollButtons(); // catch input during render to keep menu snappy
        if (btnPending != BTN_NONE) {
            if (dimmed) { ledcWrite(PIN_LCD_BL, settings[1].value); dimmed=false; }
            resetActivity();
            handleInput(btnPending);
            btnPending=BTN_NONE;
        }
    }

    yield();

    if (now-tDebug >= 5000) {
        tDebug=now;
        // Array must match UiState enum order exactly (14 entries, 0-13)
        const char* s[]={"HOME","MENU","MDL","MDL_EDIT","CH","CH_MAP","AX_REV","RATES","RADIO","PAIR","CAL","SET","TELEM","ABOUT"};
        Serial.printf("[WD] UI:%s  %.2fV %d%%  LQ:%d%%  %s  Model:%s  Heap:%luKB\n",
            (uiState < sizeof(s)/sizeof(s[0])) ? s[uiState] : "?",
            battVoltage,battPercent(),
            linkQuality,radioModeName(),models[activeModel].name,
            (unsigned long)(ESP.getFreeHeap()/1024));
    }
}
