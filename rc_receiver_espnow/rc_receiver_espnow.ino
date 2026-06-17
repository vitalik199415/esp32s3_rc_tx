// =============================================================================
// RC RECEIVER — ESP-NOW v1.0
// Board: Tenstar Robot ESP32-C3 SuperMini
//
// Features:
//   • ESP-NOW channel receive (10 channels from TX)
//   • Pairing mode — hold BOOT button 3s OR no saved TX MAC
//   • 8 PWM servo outputs
//   • Failsafe — servos return to neutral on signal loss (500ms timeout)
//   • Status LED (active LOW — GPIO8 on C3 SuperMini)
//   • MAC address saved to NVS
//
// Arduino IDE settings:
//   Board            : ESP32C3 Dev Module
//   USB CDC On Boot  : Enabled
//   Flash Size       : 4MB
//   CPU Frequency    : 160MHz
//
// Required library:
//   ESP32Servo by Kevin Harrington (latest)
//
// Tenstar C3 SuperMini pinout notes:
//   GPIO 8  = blue LED, active LOW (LOW=on, HIGH=off)
//   GPIO 9  = BOOT button (active LOW)
//   Avoid   : GPIO 18/19 (USB), GPIO 12/13 (strapping)
//
// Servo pin mapping:
//   CH1 Aileron    → SERVO_PINS[0]
//   CH2 Elevator   → SERVO_PINS[1]
//   CH3 Throttle   → SERVO_PINS[2]
//   CH4 Rudder     → SERVO_PINS[3]
//   CH5 FlightMode → SERVO_PINS[4]
//   CH6 Camera     → SERVO_PINS[5]
//   CH7 ARM        → SERVO_PINS[6]
//   CH8 RTH        → SERVO_PINS[7]
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <Preferences.h>
#include <ESP32Servo.h>

// =============================================================================
// CONFIGURATION
// =============================================================================
#define LED_PIN         8       // active LOW — LOW=on, HIGH=off
#define PAIR_BTN_PIN    9       // BOOT button, active LOW
#define SERVO_COUNT     6
#define FAILSAFE_MS     500
#define ESPNOW_CHANNEL  1       // must match TX ESPNOW_CHANNEL in config.h
#define CH_MIN          1000
#define CH_MID          1500
#define CH_MAX          2000

const uint8_t SERVO_PINS[SERVO_COUNT] = { 1, 2, 3, 4, 5, 7 };

// Channel→Servo mapping: which TX channel drives each servo output
// Index = servo slot (0..SERVO_COUNT-1), value = channel index (0..9)
// CH index: 0=AIL 1=ELE 2=THR 3=RUD 4=FM 5=CAM 6=ARM 7=RTH 8=PotA 9=PotB
// Example below: servo 0←AIL, 1←ELE, 2←THR, 3←RUD, 4←ARM, 5←RTH
const uint8_t CH_MAP[SERVO_COUNT] = { 0, 1, 2, 3, 6, 7 };

const uint16_t FAILSAFE_US[SERVO_COUNT] = {
    CH_MID,  // CH1 Aileron   — centre
    CH_MID,  // CH2 Elevator  — centre
    CH_MIN,  // CH3 Throttle  — minimum (safety!)
    CH_MID,  // CH4 Rudder    — centre
    CH_MIN,  // CH7 ARM       — disarmed
    CH_MIN,  // CH8 RTH       — off
};

// =============================================================================
// TYPES — all at top (Arduino IDE preprocessor requirement)
// =============================================================================
enum PktType : uint8_t {
    PTYPE_DATA      = 0x01,
    PTYPE_PAIR_REQ  = 0x10,
    PTYPE_PAIR_ACK  = 0x11,
    PTYPE_PAIR_DONE = 0x12,
    PTYPE_TELEMETRY = 0x20,
};

enum RxState : uint8_t { RX_NORMAL, RX_PAIRING, RX_FAILSAFE };

struct TelemetryPkt {
    PktType  type;
    int8_t   rssi;       // last received packet RSSI (dBm)
    uint8_t  lq;         // link quality 0-100%
    uint16_t rxVoltage;  // mV (0 if not measuring)
    uint8_t  rxArmed;    // 1 if ARM channel active
    int8_t   rxTemp;     // reserved
};

struct EspNowPkt {
    PktType type;
    uint8_t mac[6];
    uint8_t data[20];
};

// =============================================================================
// GLOBAL STATE
// =============================================================================
RxState  rxState      = RX_NORMAL;
bool     paired       = false;
uint8_t  txMac[6]     = {0};
uint16_t chData[10]   = {0};
uint32_t lastPktMs    = 0;

// Telemetry tracking
int8_t   lastRssi      = 0;    // RSSI of last received packet
uint8_t  pktCount      = 0;    // packets received in last second
uint8_t  linkQuality   = 0;    // 0-100%
uint32_t lqWindowMs    = 0;    // window start time for LQ calc

// Optional RX battery voltage measurement
// Connect RX battery via voltage divider to GPIO 3 (ADC)
// Use 100k+47k divider: Vbat → 100k → ADC pin → 47k → GND
// Max Vbat = 3.3 * (100+47)/47 = ~10.3V (safe for 2S LiPo)
#define RX_VBAT_PIN     3      // set -1 to disable
#define RX_VDIV_RATIO   3.128f // (100+47)/47
uint32_t pairBtnMs    = 0;
bool     pairBtnHeld  = false;

Servo       servos[SERVO_COUNT];
Preferences prefs;

// =============================================================================
// LED — active LOW
// =============================================================================
void ledOn()  { digitalWrite(LED_PIN, LOW);  }
void ledOff() { digitalWrite(LED_PIN, HIGH); }

// Blocking flash — only used at startup and pairing events
void ledFlash(uint8_t count, uint16_t onMs = 100, uint16_t offMs = 80) {
    for (uint8_t i = 0; i < count; i++) {
        ledOn();  delay(onMs);
        ledOff();
        if (i < count - 1) delay(offMs);
    }
}

// =============================================================================
// NVS
// =============================================================================
void saveTxMac() {
    prefs.begin("rc_rx", false);
    prefs.putBytes("txmac", txMac, 6);
    prefs.end();
    paired = true;
    Serial.println("[RX] TX MAC saved");
}

bool loadTxMac() {
    prefs.begin("rc_rx", true);
    size_t len = prefs.getBytesLength("txmac");
    if (len == 6) {
        prefs.getBytes("txmac", txMac, 6);
        prefs.end();
        bool allZero = true, allFF = true;
        for (int i = 0; i < 6; i++) {
            if (txMac[i] != 0x00) allZero = false;
            if (txMac[i] != 0xFF) allFF  = false;
        }
        return !allZero && !allFF;
    }
    prefs.end();
    return false;
}

void clearTxMac() {
    prefs.begin("rc_rx", false);
    prefs.remove("txmac");
    prefs.end();
    memset(txMac, 0, 6);
    paired = false;
    Serial.println("[RX] TX MAC cleared");
}

// =============================================================================
// SERVOS
// =============================================================================
void servosWrite(const uint16_t* us) {
    for (int i = 0; i < SERVO_COUNT; i++)
        servos[i].writeMicroseconds(constrain(us[i], CH_MIN, CH_MAX));
}

void servosFailsafe() { servosWrite(FAILSAFE_US); }

void servosFromChannels() {
    uint16_t us[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        uint8_t ch = CH_MAP[i];
        us[i] = (ch < 10) ? chData[ch] : CH_MID;
    }
    servosWrite(us);
}

// =============================================================================
// PAIRING
// =============================================================================
void sendTelemetry() {
    if (!paired || rxState != RX_NORMAL) return;
    // Measure RX battery voltage
    uint16_t rxVoltage = 0;
#if RX_VBAT_PIN >= 0
    uint32_t adcSum = 0;
    for (int i = 0; i < 8; i++) adcSum += analogRead(RX_VBAT_PIN);
    float adcV = (adcSum / 8.0f) * (3.3f / 4095.f);
    rxVoltage = (uint16_t)(adcV * RX_VDIV_RATIO * 1000.f);
#endif

    TelemetryPkt pkt;
    pkt.type      = PTYPE_TELEMETRY;
    pkt.rssi      = lastRssi;
    pkt.lq        = linkQuality;
    pkt.rxVoltage = rxVoltage;
    pkt.rxArmed   = (chData[6] > 1700) ? 1 : 0;
    pkt.rxTemp    = 0;

    esp_now_send(txMac, (uint8_t*)&pkt, sizeof(TelemetryPkt));
}

void enterPairingMode() {
    rxState = RX_PAIRING;
    Serial.println("[RX] *** PAIRING MODE ACTIVE — trigger pairing on TX now ***");
    Serial.println("[RX] LED will blink fast. On TX: Menu → Radio → UP button");
    // 5 fast blinks to confirm entry
    ledFlash(5, 80, 80);
}

void sendPairAck(const uint8_t* toMac) {
    EspNowPkt pkt;
    pkt.type = PTYPE_PAIR_ACK;
    uint8_t selfMac[6];
    esp_read_mac(selfMac, ESP_MAC_WIFI_STA);
    memcpy(pkt.mac, selfMac, 6);
    if (!esp_now_is_peer_exist(toMac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, toMac, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }
    esp_now_send(toMac, (uint8_t*)&pkt, 7);
    Serial.println("[RX] Sent PAIR_ACK");
}

// =============================================================================
// ESP-NOW CALLBACKS
// =============================================================================
void onRecv(const esp_now_recv_info_t* info, const uint8_t* rawData, int len) {
    if (len < 1) return;
    PktType type = (PktType)rawData[0];

    if (type == PTYPE_PAIR_REQ && rxState == RX_PAIRING && len >= 7) {
        sendPairAck(info->src_addr);

    } else if (type == PTYPE_PAIR_DONE && rxState == RX_PAIRING && len >= 7) {
        memcpy(txMac, rawData + 1, 6);
        saveTxMac();
        rxState   = RX_NORMAL;
        lastPktMs = millis() + 2000; // grace period for pairing completion
        Serial.printf("[RX] Paired! TX: %02X:%02X:%02X:%02X:%02X:%02X\n",
            txMac[0],txMac[1],txMac[2],txMac[3],txMac[4],txMac[5]);
        ledFlash(3, 150, 100);

    } else if (type == PTYPE_DATA && len >= 21) {
        // Track RSSI from packet metadata
        lastRssi = (int8_t)info->rx_ctrl->rssi;
        pktCount++;
        // Check sender is our paired TX (if paired)
        if (paired) {
            for (int i = 0; i < 6; i++)
                if (info->src_addr[i] != txMac[i]) return;
        }
        for (int i = 0; i < 10; i++)
            chData[i] = ((uint16_t)rawData[1 + i*2] << 8) | rawData[2 + i*2];
        lastPktMs = millis();
        rxState   = RX_NORMAL;
    }
}

void onSend(const wifi_tx_info_t* info, esp_now_send_status_t status) {}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== RC Receiver ESP-NOW v1.0 (C3 SuperMini) ===");

    // LED — active LOW, start OFF
    pinMode(LED_PIN,      OUTPUT);
    pinMode(PAIR_BTN_PIN, INPUT_PULLUP);
    ledOff();

    // Brief boot flash
    ledFlash(2, 100, 80);

    // Servos
    for (int i = 0; i < SERVO_COUNT; i++) {
        Serial.printf("Attach servo %d on GPIO%d\n", i, SERVO_PINS[i]);
        bool ok = servos[i].attach(SERVO_PINS[i], CH_MIN, CH_MAX);
        Serial.printf("Result = %d\n", ok);
        servos[i].writeMicroseconds(CH_MID);
    }

    // Load paired TX MAC
    paired = loadTxMac();
    if (paired) {
        Serial.printf("[RX] Known TX: %02X:%02X:%02X:%02X:%02X:%02X\n",
            txMac[0],txMac[1],txMac[2],txMac[3],txMac[4],txMac[5]);
    } else {
        Serial.println("[RX] No saved TX — will enter pairing mode");
    }

    // ESP-NOW init
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("[RX] ESP-NOW init FAILED!");
        while (1) { ledFlash(10, 50, 50); delay(500); }
    }
    esp_now_register_send_cb(onSend);
    esp_now_register_recv_cb(onRecv);

    // Always register broadcast peer (needed to receive PAIR_REQ)
    uint8_t broadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_peer_info_t bPeer = {};
    memcpy(bPeer.peer_addr, broadcast, 6);
    bPeer.channel = ESPNOW_CHANNEL;
    bPeer.encrypt = false;
    esp_now_add_peer(&bPeer);

    // Register known TX as peer
    if (paired) {
        esp_now_peer_info_t txPeer = {};
        memcpy(txPeer.peer_addr, txMac, 6);
        txPeer.channel = ESPNOW_CHANNEL;
        txPeer.encrypt = false;
        esp_now_add_peer(&txPeer);
    }

    uint8_t selfMac[6];
    esp_read_mac(selfMac, ESP_MAC_WIFI_STA);
    Serial.printf("[RX] Own MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        selfMac[0],selfMac[1],selfMac[2],selfMac[3],selfMac[4],selfMac[5]);

    // No boot delay — instant startup
    // To re-pair: hold BOOT button 2 seconds during operation
    if (!paired) {
        enterPairingMode();
    } else {
        rxState   = RX_NORMAL;
        lastPktMs = millis();
        Serial.println("[RX] Ready — hold BOOT 2s to re-pair");
    }
}

// =============================================================================
// LOOP
// =============================================================================
static uint32_t lastLedMs    = 0;
static bool     ledState     = false;
static uint32_t lastPrintMs  = 0;
static uint32_t lastTelemMs  = 0;

void loop() {
    uint32_t now = millis();

    // ── BOOT button — hold 2s during operation to re-enter pairing ──────────────
    if (!digitalRead(PAIR_BTN_PIN)) {
        if (!pairBtnHeld) {
            pairBtnMs  = now;
            pairBtnHeld = true;
            Serial.println("[RX] BOOT held — release within 2s to cancel...");
        }
        if (pairBtnHeld && (now - pairBtnMs) > 2000) {
            clearTxMac();
            enterPairingMode();
            pairBtnHeld = false;
        }
    } else {
        pairBtnHeld = false;
    }

    // ── Failsafe ──────────────────────────────────────────────────────────────
    if (rxState == RX_NORMAL && paired) {
        if ((now - lastPktMs) > FAILSAFE_MS) {
            rxState = RX_FAILSAFE;
            Serial.println("[RX] FAILSAFE");
            servosFailsafe();
        } else {
            servosFromChannels();
        }
    }
    if (rxState == RX_FAILSAFE && (now - lastPktMs) < FAILSAFE_MS) {
        rxState = RX_NORMAL;
        Serial.println("[RX] Signal restored");
    }

    // ── LED blink pattern ─────────────────────────────────────────────────────
    // PAIRING  : fast 150ms — searching
    // FAILSAFE : medium 300ms — signal lost
    // NORMAL   : slow 1200ms — all good
    uint32_t blinkRate;
    if      (rxState == RX_PAIRING)  blinkRate = 250;   // obvious fast blink
    else if (rxState == RX_FAILSAFE) blinkRate = 300;
    else                             blinkRate = 1200;

    if (now - lastLedMs > blinkRate) {
        lastLedMs = now;
        ledState  = !ledState;
        ledState ? ledOn() : ledOff();
    }

    // ── Link quality — count packets per second ──────────────────────────────
    if ((now - lqWindowMs) >= 1000) {
        const uint8_t EXPECTED = 50;
        linkQuality = (uint8_t)constrain((pktCount * 100) / EXPECTED, 0, 100);
        pktCount    = 0;
        lqWindowMs  = now;
    }

    // ── Send telemetry to TX every 500ms ─────────────────────────────────────
    if (now - lastTelemMs >= 500) {
        lastTelemMs = now;
        sendTelemetry();
    }

    // ── Serial debug every 2s ─────────────────────────────────────────────────
    if (now - lastPrintMs > 2000) {
        lastPrintMs = now;
        const char* st[] = {"NORMAL","PAIRING","FAILSAFE"};
        Serial.printf("[RX] %s  LQ:%d%%  RSSI:%ddBm  CH3:%d  ARM:%s\n",
            st[rxState], linkQuality, (int)lastRssi,
            chData[2], chData[6] > 1700 ? "ON" : "OFF");
    }

    yield();
}
