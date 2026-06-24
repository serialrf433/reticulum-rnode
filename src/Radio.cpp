// src/Radio.cpp — non-blocking SX1262 radio over RadioLib.
//
// Board differences (pins, TCXO, RF switch, VEXT) come from the pre-included
// board header macros. The radio core never busy-waits on airtime: TX is
// async (startTransmit/finishTransmit) and a poll()-driven state machine
// advances TX/RX so the nRF52 SoftDevice keeps servicing BLE mid-transmission.

#include "Radio.h"
#include "Config.h"
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

namespace rlr { namespace radio {

static const Config* s_cfg_ptr = nullptr;

static Module s_module(PIN_LORA_NSS,    // CS / NSS
                       PIN_LORA_DIO1,   // IRQ (DIO1)
                       PIN_LORA_RESET,  // RESET
                       PIN_LORA_BUSY);  // BUSY
static SX1262 s_radio(&s_module);

static bool s_online = false;

// ---- RNode air framing --------------------------------------------
static constexpr uint8_t FLAG_SPLIT   = 0x01;
static constexpr size_t  SINGLE_MTU   = 255;   // max LoRa frame (header + 254 payload)
static constexpr size_t  SINGLE_PAYLOAD = SINGLE_MTU - 1;  // 254
static constexpr size_t  MAX_PAYLOAD  = 508;   // max reassembled Reticulum payload

// ---- TX/RX state machine ------------------------------------------
// All SPI work happens in poll(); the ISR only sets s_dio1.
enum RState : uint8_t { ST_RX, ST_TX1, ST_TX2 };
static volatile RState s_state = ST_RX;
static volatile bool   s_dio1  = false;

static void isr_dio1() { s_dio1 = true; }

// Pending/in-flight TX (full Reticulum payload, no air header).
static uint8_t  s_tx_payload[512];
static size_t   s_tx_len      = 0;     // >0 => a TX is pending CSMA or in flight
static uint8_t  s_tx_header   = 0;     // shared air header for both split frames
static bool     s_tx_split    = false;
static int      s_tx_tries    = 0;
static uint32_t s_tx_next_ms  = 0;     // CSMA backoff deadline
static bool     s_tx_done_evt = false; // latched on completion, read by host layer
static size_t   s_last_tx_len = 0;

// RNode split-packet reassembly. We deliberately do NOT time out a buffered
// first half on a wall clock: at slow data rates the second half legitimately
// arrives seconds later. It is replaced when a different-sequence (or non-split)
// frame arrives — matching upstream RNode_Firmware.
static uint8_t  s_split_buf[512];
static size_t   s_split_len = 0;
static uint8_t  s_split_seq = 0xFF;    // 0xFF = no first half buffered

static RxCallback s_rx_cb = nullptr;

static float s_last_rssi = 0;
static float s_last_snr  = 0;

// ---- CSMA/CA parameters -------------------------------------------
static constexpr float CSMA_RSSI_THRESHOLD_DBM = -90.0f;
static constexpr int   CSMA_MAX_RETRIES        = 5;
static constexpr int   CSMA_SLOT_MS_MIN        = 10;
static constexpr int   CSMA_SLOT_MS_MAX        = 50;

// ---- Hardware bring-up --------------------------------------------

bool init_hardware() {
    #if HAS_VEXT_RAIL && defined(PIN_VEXT_EN) && PIN_VEXT_EN >= 0
        pinMode(PIN_VEXT_EN, OUTPUT);
        digitalWrite(PIN_VEXT_EN, HIGH);
        delay(VEXT_SETTLE_MS);
    #endif
    #if defined(RADIO_SPI_OVERRIDE_PINS) && RADIO_SPI_OVERRIDE_PINS
        SPI.setPins(PIN_LORA_MISO, PIN_LORA_SCK, PIN_LORA_MOSI);
    #endif
    SPI.begin();
    Serial.println("Radio: init_hardware OK (VEXT + SPI pins ready)");
    return true;
}

bool begin(const Config& cfg) {
    s_cfg_ptr = &cfg;
    float freq_mhz = (float)cfg.freq_hz / 1000000.0f;
    float bw_khz   = (float)cfg.bw_hz   / 1000.0f;

    const uint8_t  sync_word     = 0x12;   // RNode / private-LoRa sync word
    const uint16_t preamble_len  = 16;
    const bool use_regulator_ldo = false;

    float tcxo_v = 0.0f;
    #if HAS_TCXO && defined(RADIO_TCXO_VOLTAGE_MV)
        tcxo_v = (float)RADIO_TCXO_VOLTAGE_MV / 1000.0f;
    #endif

    int state = s_radio.begin(freq_mhz, bw_khz, (uint8_t)cfg.sf,
                              (uint8_t)cfg.cr, sync_word,
                              (int8_t)cfg.txp_dbm, preamble_len,
                              tcxo_v, use_regulator_ldo);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Radio: SX1262 begin() failed, RadioLib code ");
        Serial.println(state);
        return false;
    }

    s_radio.setCRC(1);

    #if RADIO_DIO2_AS_RF_SWITCH
        state = s_radio.setDio2AsRfSwitch(true);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.print("Radio: setDio2AsRfSwitch failed, RadioLib code ");
            Serial.println(state);
        }
    #endif

    #if defined(PIN_LORA_RXEN) && PIN_LORA_RXEN >= 0
        {
            uint32_t tx_pin = RADIOLIB_NC;
            #if defined(PIN_LORA_TXEN) && PIN_LORA_TXEN >= 0
                tx_pin = (uint32_t)PIN_LORA_TXEN;
            #endif
            s_radio.setRfSwitchPins((uint32_t)PIN_LORA_RXEN, tx_pin);
        }
    #endif

    s_radio.setRxBoostedGainMode(true);

    // Attach the single DIO1 handler. RadioLib's startReceive()/startTransmit()
    // set the IRQ mask (RxDone vs TxDone); the same handler fires for both and
    // poll() disambiguates by state.
    s_radio.setDio1Action(isr_dio1);

    s_online = true;
    Serial.print("Radio: configured @ ");
    Serial.print(cfg.freq_hz);
    Serial.print(" Hz, BW=");
    Serial.print(cfg.bw_hz);
    Serial.print(" Hz, SF=");
    Serial.print(cfg.sf);
    Serial.print(", CR=4/");
    Serial.print(cfg.cr);
    Serial.print(", TXP=");
    Serial.print(cfg.txp_dbm);
    Serial.print(" dBm, TCXO=");
    Serial.print(tcxo_v, 2);
    Serial.println(" V");
    return true;
}

void set_rx_callback(RxCallback cb) { s_rx_cb = cb; }

static void arm_rx() {
    s_state = ST_RX;
    s_radio.startReceive();
}

bool start_rx() {
    if (!s_online) {
        Serial.println("Radio: start_rx() called before begin()");
        return false;
    }
    s_dio1 = false;
    arm_rx();
    Serial.println("Radio: entering continuous RX");
    return true;
}

bool online()     { return s_online; }
float last_rssi() { return s_last_rssi; }
float last_snr()  { return s_last_snr; }
size_t last_tx_len() { return s_last_tx_len; }

void stop() {
    s_radio.standby();
    s_online = false;
}

float read_rssi() { return s_radio.getRSSI(false); }
bool  channel_clear() { return read_rssi() < CSMA_RSSI_THRESHOLD_DBM; }

void csma_params(uint8_t& cw_band, uint8_t& cw_min, uint8_t& cw_max) {
    cw_band = 1;
    cw_min  = 0;
    cw_max  = (uint8_t)CSMA_MAX_RETRIES;
}

bool tx_busy() { return s_tx_len > 0 || s_state != ST_RX; }

bool tx_just_completed() {
    if (s_tx_done_evt) { s_tx_done_evt = false; return true; }
    return false;
}

// ---- TX path ------------------------------------------------------

// Air one LoRa frame: header byte + payload. startTransmit() returns once the
// frame is in the FIFO; TxDone arrives later via DIO1 -> poll().
static bool start_frame(const uint8_t* payload, size_t plen, RState next_state) {
    uint8_t buf[256];
    if (plen + 1 > sizeof(buf)) return false;
    buf[0] = s_tx_header;
    memcpy(buf + 1, payload, plen);
    s_dio1 = false;
    int st = s_radio.startTransmit(buf, plen + 1);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.print("Radio: startTransmit error code=");
        Serial.println(st);
        return false;
    }
    s_state = next_state;
    return true;
}

static void begin_tx_frames() {
    size_t first = s_tx_split ? SINGLE_PAYLOAD : s_tx_len;
    if (!start_frame(s_tx_payload, first, ST_TX1)) {
        // Recover into RX and drop the frame; higher layers retransmit.
        s_tx_len = 0;
        arm_rx();
    }
}

static void complete_tx() {
    s_last_tx_len = s_tx_len;
    s_tx_len = 0;
    s_tx_done_evt = true;
    arm_rx();
}

bool enqueue_tx(const uint8_t* buf, size_t len) {
    if (!s_online) return false;
    if (s_tx_len > 0 || s_state != ST_RX) return false;  // busy
    if (s_dio1) return false;   // an RX event is pending; let poll() service it first
    if (len == 0 || len > MAX_PAYLOAD) return false;

    memcpy(s_tx_payload, buf, len);
    s_tx_len    = len;
    s_tx_header = (uint8_t)(random(256) & 0xF0);
    s_tx_split  = (len > SINGLE_PAYLOAD);
    if (s_tx_split) s_tx_header |= FLAG_SPLIT;
    s_tx_tries   = 0;
    s_tx_next_ms = millis();   // attempt CSMA on the next poll()
    return true;
}

// ---- RX path ------------------------------------------------------

static void deliver_rx() {
    size_t len = s_radio.getPacketLength();
    if (len == 0) { arm_rx(); return; }

    uint8_t rx_tmp[512];
    if (len > sizeof(rx_tmp)) { arm_rx(); return; }

    int st = s_radio.readData(rx_tmp, len);
    s_last_rssi = s_radio.getRSSI();
    s_last_snr  = s_radio.getSNR();
    if (st != RADIOLIB_ERR_NONE || len < 2) { arm_rx(); return; }

    uint8_t  hdr = rx_tmp[0];
    uint8_t  seq = (hdr >> 4) & 0x0F;
    bool     is_split = (hdr & FLAG_SPLIT) != 0;
    uint8_t* payload  = rx_tmp + 1;
    size_t   plen     = len - 1;

    if (is_split) {
        if (s_split_seq == 0xFF) {
            // First half — buffer it.
            if (plen <= sizeof(s_split_buf)) {
                memcpy(s_split_buf, payload, plen);
                s_split_len = plen;
                s_split_seq = seq;
            }
            arm_rx();
            return;
        } else if (seq == s_split_seq) {
            // Second half — join.
            size_t total = s_split_len + plen;
            s_split_seq = 0xFF;
            if (total <= MAX_PAYLOAD) {
                uint8_t out[512];
                memcpy(out, s_split_buf, s_split_len);
                memcpy(out + s_split_len, payload, plen);
                s_split_len = 0;
                if (s_rx_cb) s_rx_cb(out, total, s_last_rssi, s_last_snr);
            } else {
                s_split_len = 0;
            }
            arm_rx();
            return;
        } else {
            // Different sequence — replace the buffered first half.
            if (plen <= sizeof(s_split_buf)) {
                memcpy(s_split_buf, payload, plen);
                s_split_len = plen;
                s_split_seq = seq;
            } else {
                s_split_seq = 0xFF;
                s_split_len = 0;
            }
            arm_rx();
            return;
        }
    }

    // Non-split: a buffered first half is now stale — drop it and deliver.
    s_split_seq = 0xFF;
    s_split_len = 0;
    if (s_rx_cb) s_rx_cb(payload, plen, s_last_rssi, s_last_snr);
    arm_rx();
}

// ---- State machine ------------------------------------------------

void poll() {
    if (!s_online) return;

    // Pending TX awaiting a clear channel: carrier-sense without blocking.
    if (s_state == ST_RX && s_tx_len > 0 && !s_dio1 &&
        (int32_t)(millis() - s_tx_next_ms) >= 0) {
        if (channel_clear() || s_tx_tries >= CSMA_MAX_RETRIES) {
            begin_tx_frames();
        } else {
            s_tx_tries++;
            s_tx_next_ms = millis() + CSMA_SLOT_MS_MIN +
                           random(CSMA_SLOT_MS_MAX - CSMA_SLOT_MS_MIN);
        }
    }

    if (!s_dio1) return;
    s_dio1 = false;

    switch (s_state) {
        case ST_TX1:
            s_radio.finishTransmit();
            if (s_tx_split) {
                // Air the second half with the same header byte.
                if (!start_frame(s_tx_payload + SINGLE_PAYLOAD,
                                 s_tx_len - SINGLE_PAYLOAD, ST_TX2)) {
                    s_tx_len = 0;
                    arm_rx();
                }
            } else {
                complete_tx();
            }
            break;

        case ST_TX2:
            s_radio.finishTransmit();
            complete_tx();
            break;

        case ST_RX:
        default:
            deliver_rx();
            break;
    }
}

}} // namespace rlr::radio
