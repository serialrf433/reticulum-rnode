// src/Kiss.cpp — KISS frame encoder/decoder for RNode serial protocol.

#include "Kiss.h"
#include "Radio.h"
#include "Led.h"
#include "Eeprom.h"
#include "Battery.h"
#include "Ble.h"
#include "Airtime.h"
#include "KissCodec.h"
#include <Arduino.h>
#include <math.h>
#include <nrf_soc.h>
#include <InternalFileSystem.h>

#ifndef RLR_VERSION
  #define RLR_VERSION "0.1.0-dev"
#endif

namespace rlr { namespace kiss {

// ---- State -------------------------------------------------------

static constexpr size_t FRAME_BUF_SIZE = 512;

// Serial KISS parser state
static uint8_t s_frame_buf[FRAME_BUF_SIZE];
static size_t  s_frame_len = 0;
static bool    s_in_frame  = false;
static bool    s_escape    = false;


// Current radio config (set by host commands)
static uint32_t s_freq_hz   = 0;
static uint32_t s_bw_hz     = 0;
static uint8_t  s_sf        = 0;
static uint8_t  s_cr        = 0;
static int8_t   s_txp_dbm   = 0;
static bool     s_radio_on  = false;

// Radio config lock (CMD_RADIO_LOCK) and implicit-header mode
// (CMD_IMPLICIT). When locked, radio parameter changes are refused.
static bool s_radio_locked = false;
static bool s_implicit     = false;

// LoRa preamble length actually programmed by Radio.cpp; used for the
// airtime and PHYPRM timing math so the reported figures match the air.
static constexpr uint16_t PREAMBLE_SYMBOLS = 16;

// Airtime limits (CMD_ST_ALOCK / CMD_LT_ALOCK), as a percentage of the
// respective window. 0 = no limit (default — host only sends these when
// the operator has configured an airtime cap).
static double s_st_alock_pct = 0.0;
static double s_lt_alock_pct = 0.0;

// Rolling airtime/channel accounting (SPEC.md §8.5). s_tx_airtime is our
// own transmit occupancy (drives the ALOCK caps); s_channel_busy adds
// carrier-sense-detected traffic for the CMD_STAT_CHTM channel-load
// figures.
static rlr::airtime::UtilTracker s_tx_airtime;
static rlr::airtime::UtilTracker s_channel_busy;

// Channel RSSI / noise-floor estimates sampled while the radio is idle.
static int   s_current_rssi = -157;
static int   s_noise_floor  = -157;
static float s_nf_min       = 0.0f;
static bool  s_nf_seeded    = false;

// Periodic status-emission / channel-sampling timers.
static uint32_t s_last_status_ms = 0;
static uint32_t s_last_sample_ms = 0;
static constexpr uint32_t STATUS_INTERVAL_MS = 2000;
static constexpr uint32_t SAMPLE_INTERVAL_MS = 100;

// Firmware version (must be >= 1.52 for Reticulum host)
static constexpr uint8_t FW_MAJ = 1;
static constexpr uint8_t FW_MIN = 52;

// Packet counters
static uint32_t s_rx_count = 0;
static uint32_t s_tx_count = 0;

// ROM lock state (must be unlocked by host before ROM_WRITE)
static bool s_rom_unlocked = false;

// Standard RNode EEPROM address map (matches rnodeconf / webflasher)
static constexpr uint16_t ADDR_PRODUCT   = 0x00;  // 1 byte
static constexpr uint16_t ADDR_MODEL     = 0x01;  // 1 byte
static constexpr uint16_t ADDR_HW_REV    = 0x02;  // 1 byte
static constexpr uint16_t ADDR_SERIAL    = 0x03;  // 4 bytes
static constexpr uint16_t ADDR_MADE      = 0x07;  // 4 bytes
static constexpr uint16_t ADDR_CHKSUM    = 0x0B;  // 16 bytes (MD5)
static constexpr uint16_t ADDR_SIGNATURE = 0x1B;  // 128 bytes
static constexpr uint16_t ADDR_INFO_LOCK = 0x9B;  // 1 byte (0x73 = locked)
static constexpr uint16_t ADDR_CONF_SF   = 0x9C;  // 1 byte
static constexpr uint16_t ADDR_CONF_CR   = 0x9D;  // 1 byte
static constexpr uint16_t ADDR_CONF_TXP  = 0x9E;  // 1 byte
static constexpr uint16_t ADDR_CONF_BW   = 0x9F;  // 4 bytes
static constexpr uint16_t ADDR_CONF_FREQ = 0xA3;  // 4 bytes
static constexpr uint16_t ADDR_CONF_OK   = 0xA7;  // 1 byte (0x73 = valid)
static constexpr uint16_t ADDR_FW_HASH   = 0xB0;  // 32 bytes

// TX queue — single-slot buffer. When the radio is busy transmitting,
// the next packet waits here. CMD_READY reports queue availability.
static constexpr size_t TX_QUEUE_SIZE = 512;
static uint8_t  s_tx_queue[TX_QUEUE_SIZE];
static size_t   s_tx_queue_len = 0;
static bool     s_tx_busy = false;

// ---- Transport abstraction ----------------------------------------
// Tracks which transport (Serial or BLE) sent the last command so
// responses go back the same way. RX packets always go to both.

enum Transport : uint8_t { TRANSPORT_SERIAL = 0, TRANSPORT_BLE = 1 };
static Transport s_last_transport = TRANSPORT_SERIAL;

static void _write_byte(uint8_t b, Transport t) {
    if (t == TRANSPORT_BLE) {
        rlr::ble::write(b);
    } else {
        Serial.write(b);
    }
}

static void _write_buf(const uint8_t* buf, size_t len, Transport t) {
    if (t == TRANSPORT_BLE) {
        rlr::ble::write(buf, len);
    } else {
        Serial.write(buf, len);
    }
}

// ---- KISS frame encoder ------------------------------------------

// Build a KISS frame into a buffer and send in one write.
static constexpr size_t KISS_TX_BUF_SIZE = 1100;
static uint8_t s_kiss_tx_buf[KISS_TX_BUF_SIZE];

static void _send_frame_on(uint8_t cmd, const uint8_t* data, size_t len, Transport t) {
    size_t pos = 0;
    s_kiss_tx_buf[pos++] = FEND;
    s_kiss_tx_buf[pos++] = cmd;
    // Escape the payload with the shared, unit-tested KISS codec. Reserve
    // one byte for the trailing FEND.
    pos += rlr::kisscodec::escape(data, len, s_kiss_tx_buf + pos, KISS_TX_BUF_SIZE - pos - 1);
    s_kiss_tx_buf[pos++] = FEND;
    _write_buf(s_kiss_tx_buf, pos, t);
    // Flush BLE TXD buffer at end of each KISS frame so the
    // complete frame goes out as one (or few) BLE notifications.
    if (t == TRANSPORT_BLE) {
        rlr::ble::flush();
    }
}

// Determine the active transport: when BLE is connected, all IO
// goes exclusively to BLE (matching official RNode firmware behavior).
static Transport _active_transport() {
    return rlr::ble::connected() ? TRANSPORT_BLE : TRANSPORT_SERIAL;
}

void send_frame(uint8_t cmd, const uint8_t* data, size_t len) {
    _send_frame_on(cmd, data, len, _active_transport());
}

void send_byte(uint8_t cmd, uint8_t value) {
    send_frame(cmd, &value, 1);
}

void send_rx_packet(const uint8_t* data, size_t len, float rssi, float snr) {
    // Send RX packet to the active transport (BLE if connected, else Serial)
    Transport t = _active_transport();
    uint8_t rssi_byte = (uint8_t)((int)rssi + RSSI_OFFSET);
    int8_t snr_raw = (int8_t)(snr * 4.0f);

    _send_frame_on(CMD_STAT_RSSI, &rssi_byte, 1, t);
    _send_frame_on(CMD_STAT_SNR, (uint8_t*)&snr_raw, 1, t);
    _send_frame_on(CMD_DATA, data, len, t);
    s_rx_count++;

    // Yield to FreeRTOS scheduler so the SoftDevice can process
    // BLE connection events between RX packets
    if (t == TRANSPORT_BLE) {
        delay(1);
    }
}

// ---- Helpers -----------------------------------------------------

static void apply_radio_config() {
    if (s_freq_hz == 0 || s_bw_hz == 0 || s_sf == 0 || s_cr == 0) return;

    rlr::Config cfg;
    cfg.freq_hz = s_freq_hz;
    cfg.bw_hz   = s_bw_hz;
    cfg.sf      = s_sf;
    cfg.cr      = s_cr;
    cfg.txp_dbm = s_txp_dbm;
    cfg.flags   = 0;

    rlr::radio::begin(cfg);
}

// ---- Airtime accounting & status emission (SPEC.md §8.4.5 / §8.5) --

// True if a configured airtime cap is currently exceeded, in which case
// the firmware must refuse to transmit (reporting ERROR_QUEUE_FULL).
static bool airtime_cap_blocks_tx() {
    uint32_t now = millis();
    if (s_st_alock_pct > 0.0 && s_tx_airtime.short_util(now) * 100.0 >= s_st_alock_pct) return true;
    if (s_lt_alock_pct > 0.0 && s_tx_airtime.long_util(now)  * 100.0 >= s_lt_alock_pct) return true;
    return false;
}

// Account the on-air time of a packet we just transmitted.
static void record_tx_airtime(size_t payload_len) {
    if (s_bw_hz == 0 || s_sf == 0 || s_cr == 0) return;
    uint32_t at = rlr::airtime::rnode_packet_airtime_ms(s_bw_hz, s_sf, s_cr,
                                                        PREAMBLE_SYMBOLS, payload_len);
    uint32_t now = millis();
    s_tx_airtime.record(now, at);
    s_channel_busy.record(now, at);  // our own TX occupies the channel too
}

// CMD_STAT_PHYPRM (0x26): report the current physical-layer timing.
static void emit_phyprm() {
    if (s_bw_hz == 0 || s_sf == 0) return;
    double tsym = rlr::airtime::symbol_time_ms(s_bw_hz, s_sf);
    uint16_t slot = (uint16_t)ceil(tsym);
    if (slot < 1) slot = 1;
    uint16_t difs = (uint16_t)(slot * 2 + 1);
    uint8_t buf[12];
    rlr::airtime::build_phyprm(buf, s_bw_hz, s_sf, PREAMBLE_SYMBOLS, slot, difs);
    send_frame(CMD_STAT_PHYPRM, buf, sizeof(buf));
}

// CMD_STAT_CHTM (0x25): report channel-time / airtime metrics.
static void emit_chtm() {
    uint32_t now = millis();
    uint8_t buf[11];
    rlr::airtime::build_chtm(buf,
        s_tx_airtime.short_util(now), s_tx_airtime.long_util(now),
        s_channel_busy.short_util(now), s_channel_busy.long_util(now),
        s_current_rssi, s_noise_floor, 0xFF);
    send_frame(CMD_STAT_CHTM, buf, sizeof(buf));
}

// CMD_STAT_CSMA (0x28): report the CSMA contention-window descriptor.
static void emit_csma() {
    uint8_t band, cw_min, cw_max;
    rlr::radio::csma_params(band, cw_min, cw_max);
    uint8_t buf[3];
    rlr::airtime::build_csma(buf, band, cw_min, cw_max);
    send_frame(CMD_STAT_CSMA, buf, sizeof(buf));
}

// Echo both airtime-limit settings back to the host (host caches them
// from the reply frames).
static void emit_alock() {
    uint8_t b[2];
    uint16_t st = rlr::airtime::encode_alock(s_st_alock_pct);
    b[0] = st >> 8; b[1] = st & 0xFF;
    send_frame(CMD_ST_ALOCK, b, 2);
    uint16_t lt = rlr::airtime::encode_alock(s_lt_alock_pct);
    b[0] = lt >> 8; b[1] = lt & 0xFF;
    send_frame(CMD_LT_ALOCK, b, 2);
}

// Periodic housekeeping: sample the channel for the load/noise figures
// and emit CMD_STAT_CHTM on a fixed cadence. Called from tick().
static void status_tick() {
    if (!s_radio_on) return;
    uint32_t now = millis();

    if (!s_tx_busy && (now - s_last_sample_ms) >= SAMPLE_INTERVAL_MS) {
        uint32_t dt = now - s_last_sample_ms;
        s_last_sample_ms = now;
        float rssi = rlr::radio::read_rssi();
        s_current_rssi = (int)rssi;
        if (!s_nf_seeded || rssi < s_nf_min) { s_nf_min = rssi; s_nf_seeded = true; }
        if (!rlr::radio::channel_clear()) {
            s_channel_busy.record(now, dt);   // channel sensed busy this slot
        }
    }

    if ((now - s_last_status_ms) >= STATUS_INTERVAL_MS) {
        s_last_status_ms = now;
        s_noise_floor = s_nf_seeded ? (int)s_nf_min : s_current_rssi;
        s_nf_min = (float)s_current_rssi;   // reset window minimum
        emit_chtm();
    }
}

// ---- Config persistence ------------------------------------------

static const char* CONF_FILE = "/radio_conf.dat";

// Saved config layout: [freq_hz(4) bw_hz(4) sf(1) cr(1) txp_dbm(1)] = 11 bytes
static constexpr size_t CONF_SIZE = 11;

static void save_config() {
    using namespace Adafruit_LittleFS_Namespace;
    uint8_t buf[CONF_SIZE];
    buf[0] = (s_freq_hz >> 24) & 0xFF;
    buf[1] = (s_freq_hz >> 16) & 0xFF;
    buf[2] = (s_freq_hz >>  8) & 0xFF;
    buf[3] = (s_freq_hz >>  0) & 0xFF;
    buf[4] = (s_bw_hz >> 24) & 0xFF;
    buf[5] = (s_bw_hz >> 16) & 0xFF;
    buf[6] = (s_bw_hz >>  8) & 0xFF;
    buf[7] = (s_bw_hz >>  0) & 0xFF;
    buf[8] = s_sf;
    buf[9] = s_cr;
    buf[10] = (uint8_t)s_txp_dbm;
    File f(InternalFS);
    if (f.open(CONF_FILE, FILE_O_WRITE)) {
        f.seek(0);
        f.write(buf, CONF_SIZE);
        f.close();
        Serial.println("KISS: config saved to flash");
    }
}

static bool load_config() {
    using namespace Adafruit_LittleFS_Namespace;
    File f(InternalFS);
    if (!f.open(CONF_FILE, FILE_O_READ)) return false;
    uint8_t buf[CONF_SIZE];
    size_t n = f.read(buf, CONF_SIZE);
    f.close();
    if (n < CONF_SIZE) return false;
    s_freq_hz  = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] << 8)  | (uint32_t)buf[3];
    s_bw_hz    = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16) |
                 ((uint32_t)buf[6] << 8)  | (uint32_t)buf[7];
    s_sf       = buf[8];
    s_cr       = buf[9];
    s_txp_dbm  = (int8_t)buf[10];
    Serial.println("KISS: config loaded from flash");
    return true;
}

static void delete_config() {
    InternalFS.remove(CONF_FILE);
    Serial.println("KISS: config deleted from flash");
}

static void send_uint32(uint8_t cmd, uint32_t value) {
    uint8_t buf[4];
    buf[0] = (value >> 24) & 0xFF;
    buf[1] = (value >> 16) & 0xFF;
    buf[2] = (value >>  8) & 0xFF;
    buf[3] = (value >>  0) & 0xFF;
    send_frame(cmd, buf, 4);
}

// ---- Command dispatch --------------------------------------------

static void dispatch_frame(uint8_t cmd, const uint8_t* data, size_t len) {
    switch (cmd) {

    case CMD_DATA:
        // TX packet via radio
        if (s_radio_on && len > 0) {
            // Airtime cap (CMD_ST_ALOCK/CMD_LT_ALOCK) — refuse to transmit
            // while over budget and report backpressure (SPEC.md §8.5.1).
            if (airtime_cap_blocks_tx()) {
                send_byte(CMD_ERROR, ERROR_QUEUE_FULL);
                break;
            }
            if (s_tx_busy) {
                // Radio is busy — queue the packet if the slot is free
                if (s_tx_queue_len == 0 && len <= TX_QUEUE_SIZE) {
                    memcpy(s_tx_queue, data, len);
                    s_tx_queue_len = len;
                } else {
                    send_byte(CMD_ERROR, ERROR_QUEUE_FULL);
                }
            } else {
                s_tx_busy = true;
                rlr::led::on();
                int n = rlr::radio::transmit(data, len);
                rlr::led::off();
                s_tx_busy = false;
                if (n > 0) {
                    s_tx_count++;
                    record_tx_airtime(len);
                } else {
                    send_byte(CMD_ERROR, ERROR_TXFAILED);
                }
                // Signal ready for next packet
                send_byte(CMD_READY, s_tx_queue_len == 0 ? 0x01 : 0x00);
            }
        }
        break;

    case CMD_DETECT:
        if (len >= 1 && data[0] == DETECT_REQ) {
            send_byte(CMD_DETECT, DETECT_RESP);
        }
        break;

    case CMD_FW_VERSION:
        if (len >= 1 && data[0] == 0x00) {
            uint8_t ver[2] = { FW_MAJ, FW_MIN };
            send_frame(CMD_FW_VERSION, ver, 2);
        }
        break;

    case CMD_PLATFORM:
        if (len >= 1 && data[0] == 0x00) {
            send_byte(CMD_PLATFORM, PLATFORM_NRF52);
        }
        break;

    case CMD_MCU:
        if (len >= 1 && data[0] == 0x00) {
            send_byte(CMD_MCU, MCU_NRF52);
        }
        break;

    case CMD_BOARD:
        // Respond with board model byte from header
        #if defined(BOARD_Faketec)
            send_byte(CMD_BOARD, 0x52);
        #elif defined(BOARD_RAK4631)
            send_byte(CMD_BOARD, 0x51);
        #else
            send_byte(CMD_BOARD, 0xFF);
        #endif
        break;

    case CMD_FREQUENCY:
        if (len == 4 && !s_radio_locked) {
            s_freq_hz = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                        ((uint32_t)data[2] << 8)  | (uint32_t)data[3];
            send_uint32(CMD_FREQUENCY, s_freq_hz);
        } else if (len >= 1 && data[0] == 0x00) {
            send_uint32(CMD_FREQUENCY, s_freq_hz);
        }
        break;

    case CMD_BANDWIDTH:
        if (len == 4 && !s_radio_locked) {
            s_bw_hz = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                      ((uint32_t)data[2] << 8)  | (uint32_t)data[3];
            send_uint32(CMD_BANDWIDTH, s_bw_hz);
        } else if (len >= 1 && data[0] == 0x00) {
            send_uint32(CMD_BANDWIDTH, s_bw_hz);
        }
        break;

    case CMD_TXPOWER:
        if (len == 1 && data[0] != 0xFF && !s_radio_locked) {
            s_txp_dbm = (int8_t)data[0];
            send_byte(CMD_TXPOWER, (uint8_t)s_txp_dbm);
        } else if (len >= 1 && data[0] == 0xFF) {
            send_byte(CMD_TXPOWER, (uint8_t)s_txp_dbm);
        }
        break;

    case CMD_SF:
        if (len == 1 && data[0] != 0xFF && !s_radio_locked) {
            s_sf = data[0];
            send_byte(CMD_SF, s_sf);
        } else if (len >= 1 && data[0] == 0xFF) {
            send_byte(CMD_SF, s_sf);
        }
        break;

    case CMD_CR:
        if (len == 1 && data[0] != 0xFF && !s_radio_locked) {
            s_cr = data[0];
            send_byte(CMD_CR, s_cr);
        } else if (len >= 1 && data[0] == 0xFF) {
            send_byte(CMD_CR, s_cr);
        }
        break;

    case CMD_RADIO_LOCK:
        // Lock/unlock radio config (SPEC.md §8.4.1). Non-zero = locked.
        if (len >= 1) {
            s_radio_locked = (data[0] != 0x00);
            send_byte(CMD_RADIO_LOCK, s_radio_locked ? 0x01 : 0x00);
        }
        break;

    case CMD_IMPLICIT:
        // Toggle implicit-header LoRa mode (advanced). Stored and echoed;
        // peers in this mesh use explicit headers.
        if (len >= 1) {
            s_implicit = (data[0] != 0x00);
            send_byte(CMD_IMPLICIT, s_implicit ? 0x01 : 0x00);
        }
        break;

    case CMD_ST_ALOCK:
        // Short-term airtime limit: uint16 BE of (percent * 100).
        if (len == 2) {
            uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
            s_st_alock_pct = rlr::airtime::decode_alock(raw);
            send_frame(CMD_ST_ALOCK, data, 2);   // echo back
        }
        break;

    case CMD_LT_ALOCK:
        // Long-term airtime limit: same encoding as ST_ALOCK.
        if (len == 2) {
            uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
            s_lt_alock_pct = rlr::airtime::decode_alock(raw);
            send_frame(CMD_LT_ALOCK, data, 2);   // echo back
        }
        break;

    case CMD_STAT_PHYPRM:
        // Report physical-layer timing parameters on request.
        emit_phyprm();
        break;

    case CMD_STAT_CHTM:
        // Report channel-time / airtime metrics on request.
        emit_chtm();
        break;

    case CMD_STAT_CSMA:
        // Report the CSMA contention-window descriptor on request.
        emit_csma();
        break;

    case CMD_RADIO_STATE:
        if (len == 1) {
            if (data[0] == 0x01) {
                // Turn radio ON
                if (s_freq_hz > 0 && s_bw_hz > 0 && s_sf > 0 && s_cr > 0) {
                    apply_radio_config();
                    rlr::radio::start_rx();
                    s_radio_on = true;
                    send_byte(CMD_RADIO_STATE, 0x01);
                    // Report physical params and airtime limits now that
                    // the radio is operational (SPEC.md §8.4.5 / §8.5).
                    emit_phyprm();
                    emit_csma();
                    emit_alock();
                    uint32_t now = millis();
                    s_last_status_ms = now;
                    s_last_sample_ms = now;
                } else {
                    send_byte(CMD_ERROR, ERROR_INITRADIO);
                }
            } else if (data[0] == 0x00) {
                // Turn radio OFF
                rlr::radio::stop();
                s_radio_on = false;
                send_byte(CMD_RADIO_STATE, 0x00);
            } else if (data[0] == 0xFF) {
                // Query
                send_byte(CMD_RADIO_STATE, s_radio_on ? 0x01 : 0x00);
            }
        }
        break;

    case CMD_STAT_RX:
        send_uint32(CMD_STAT_RX, s_rx_count);
        break;

    case CMD_STAT_TX:
        send_uint32(CMD_STAT_TX, s_tx_count);
        break;

    case CMD_STAT_BAT:
        send_byte(CMD_STAT_BAT, rlr::battery::level_percent());
        break;

    case CMD_READY:
        send_byte(CMD_READY, (!s_tx_busy && s_tx_queue_len == 0) ? 0x01 : 0x00);
        break;

    case CMD_BLINK:
        if (len >= 1) {
            for (uint8_t i = 0; i < data[0]; i++) {
                rlr::led::on();
                delay(100);
                rlr::led::off();
                delay(100);
            }
        }
        break;

    case CMD_RANDOM:
        {
            uint8_t rnd;
            sd_rand_application_vector_get(&rnd, 1);
            send_byte(CMD_RANDOM, rnd);
        }
        break;

    case CMD_RESET:
        if (len >= 1 && data[0] == 0xF8) {
            Serial.flush();
            delay(50);
            NVIC_SystemReset();
        }
        break;

    case CMD_ROM_READ:
        // Read from EEPROM: payload = [addr_hi, addr_lo, length]
        if (len == 3) {
            uint16_t addr = ((uint16_t)data[0] << 8) | data[1];
            uint8_t rlen = data[2];
            uint8_t rom_buf[256];
            if (rlen > sizeof(rom_buf)) rlen = sizeof(rom_buf);
            size_t n = rlr::eeprom::read(addr, rom_buf, rlen);
            send_frame(CMD_ROM_READ, rom_buf, n);
        }
        break;

    case CMD_ROM_WRITE:
        // Write to EEPROM: payload = [addr_hi, addr_lo, data...]
        if (len >= 3 && s_rom_unlocked) {
            uint16_t addr = ((uint16_t)data[0] << 8) | data[1];
            rlr::eeprom::write(addr, data + 2, len - 2);
        } else if (!s_rom_unlocked) {
            send_byte(CMD_ERROR, ERROR_EEPROM_LOCK);
        }
        break;

    case CMD_UNLOCK_ROM:
        if (len >= 1 && data[0] == 0xF8) {
            s_rom_unlocked = true;
        }
        break;

    case CMD_DEV_HASH:
        {
            // Device identity = product(1) + model(1) + hw_rev(1) + serial(4) + made(4) = 11 bytes
            // Checksum (MD5 of identity) is at ADDR_CHKSUM, 16 bytes
            uint8_t chksum[16];
            rlr::eeprom::read(ADDR_CHKSUM, chksum, 16);
            send_frame(CMD_DEV_HASH, chksum, 16);
        }
        break;

    case CMD_DEV_SIG:
        {
            // Device signature at ADDR_SIGNATURE, 128 bytes
            uint8_t sig[128];
            rlr::eeprom::read(ADDR_SIGNATURE, sig, 128);
            send_frame(CMD_DEV_SIG, sig, 128);
        }
        break;

    case CMD_FW_HASH:
        {
            // Firmware hash at ADDR_FW_HASH, 32 bytes
            uint8_t fwhash[32];
            rlr::eeprom::read(ADDR_FW_HASH, fwhash, 32);
            send_frame(CMD_FW_HASH, fwhash, 32);
        }
        break;

    case CMD_HASHES:
        {
            // Combined: checksum(16) + fw_hash(32) = 48 bytes
            uint8_t hashes[48];
            rlr::eeprom::read(ADDR_CHKSUM, hashes, 16);
            rlr::eeprom::read(ADDR_FW_HASH, hashes + 16, 32);
            send_frame(CMD_HASHES, hashes, 48);
        }
        break;

    case CMD_CONF_SAVE:
        save_config();
        break;

    case CMD_CONF_DELETE:
        delete_config();
        break;

    case CMD_CFG_READ:
        {
            // Return current radio config as a blob
            uint8_t cfg_buf[CONF_SIZE];
            cfg_buf[0] = (s_freq_hz >> 24) & 0xFF;
            cfg_buf[1] = (s_freq_hz >> 16) & 0xFF;
            cfg_buf[2] = (s_freq_hz >>  8) & 0xFF;
            cfg_buf[3] = (s_freq_hz >>  0) & 0xFF;
            cfg_buf[4] = (s_bw_hz >> 24) & 0xFF;
            cfg_buf[5] = (s_bw_hz >> 16) & 0xFF;
            cfg_buf[6] = (s_bw_hz >>  8) & 0xFF;
            cfg_buf[7] = (s_bw_hz >>  0) & 0xFF;
            cfg_buf[8] = s_sf;
            cfg_buf[9] = s_cr;
            cfg_buf[10] = (uint8_t)s_txp_dbm;
            send_frame(CMD_CFG_READ, cfg_buf, CONF_SIZE);
        }
        break;

    case CMD_BLE_PIN:
        // Read/write the BLE pairing PIN stored in EEPROM.
        // Query: 1 byte payload = 0x00 → returns current PIN (6 ASCII digits).
        // Write: 6 bytes ASCII digits → stores new PIN (takes effect after reboot).
        if (len == 1 && data[0] == 0x00) {
            uint8_t pin_buf[BLE_PIN_LEN];
            rlr::eeprom::read(ADDR_BLE_PIN, pin_buf, BLE_PIN_LEN);
            bool pin_valid = true;
            for (size_t i = 0; i < BLE_PIN_LEN; i++) {
                if (pin_buf[i] < '0' || pin_buf[i] > '9') { pin_valid = false; break; }
            }
            if (!pin_valid) memcpy(pin_buf, "123456", BLE_PIN_LEN);
            send_frame(CMD_BLE_PIN, pin_buf, BLE_PIN_LEN);
        } else if (len == BLE_PIN_LEN) {
            bool all_digits = true;
            for (size_t i = 0; i < BLE_PIN_LEN; i++) {
                if (data[i] < '0' || data[i] > '9') { all_digits = false; break; }
            }
            if (all_digits) {
                rlr::eeprom::write(ADDR_BLE_PIN, data, BLE_PIN_LEN);
                send_frame(CMD_BLE_PIN, data, BLE_PIN_LEN);  // ack with stored value
            } else {
                send_byte(CMD_ERROR, ERROR_EEPROM_LOCK);  // reuse: invalid payload
            }
        }
        break;

    case CMD_LEAVE:
        // Host disconnecting — nothing to do
        break;

    default:
        // Unknown command — ignore silently
        break;
    }
}

// ---- KISS frame parser -------------------------------------------

void init() {
    s_frame_len = 0;
    s_in_frame  = false;
    s_escape    = false;
    s_radio_on  = false;

    // Try to load saved radio config from flash
    if (load_config()) {
        Serial.print("KISS: restored config freq=");
        Serial.print(s_freq_hz);
        Serial.print(" bw=");
        Serial.print(s_bw_hz);
        Serial.print(" sf=");
        Serial.print(s_sf);
        Serial.print(" cr=");
        Serial.print(s_cr);
        Serial.print(" txp=");
        Serial.println(s_txp_dbm);
    }

    Serial.println("KISS: ready");
}

void tick() {
    // Exclusive transport: when BLE is connected, read from BLE only.
    // When disconnected, read from Serial only. This matches the
    // official RNode firmware behavior.
    bool ble_active = rlr::ble::connected();

    if (ble_active) {
        // Read from BLE
        while (rlr::ble::available()) {
            int raw = rlr::ble::read();
            if (raw < 0) break;
            uint8_t c = (uint8_t)raw;

            if (c == FEND) {
                if (s_in_frame && s_frame_len > 0) {
                    uint8_t cmd = s_frame_buf[0];
                    dispatch_frame(cmd, s_frame_buf + 1, s_frame_len - 1);
                    s_frame_len = 0;
                }
                s_in_frame = true;
                s_frame_len = 0;
                s_escape = false;
                continue;
            }
            if (!s_in_frame) continue;
            if (s_escape) {
                s_escape = false;
                if (c == TFEND) c = FEND;
                else if (c == TFESC) c = FESC;
            } else if (c == FESC) {
                s_escape = true;
                continue;
            }
            if (s_frame_len < FRAME_BUF_SIZE) {
                s_frame_buf[s_frame_len++] = c;
            }
        }
    } else {
        // Read from Serial
        while (Serial.available()) {
            uint8_t c = Serial.read();

            if (c == FEND) {
                if (s_in_frame && s_frame_len > 0) {
                    uint8_t cmd = s_frame_buf[0];
                    dispatch_frame(cmd, s_frame_buf + 1, s_frame_len - 1);
                    s_frame_len = 0;
                }
                s_in_frame = true;
                s_frame_len = 0;
                s_escape = false;
                continue;
            }
            if (!s_in_frame) continue;
            if (s_escape) {
                s_escape = false;
                if (c == TFEND) c = FEND;
                else if (c == TFESC) c = FESC;
            } else if (c == FESC) {
                s_escape = true;
                continue;
            }
            if (s_frame_len < FRAME_BUF_SIZE) {
                s_frame_buf[s_frame_len++] = c;
            }
        }
    }

    // Periodic channel sampling + CMD_STAT_CHTM emission.
    status_tick();
}

void drain_tx_queue() {
    if (s_tx_queue_len == 0 || s_tx_busy || !s_radio_on) return;
    // Hold the queued packet while over the airtime budget (backpressure).
    if (airtime_cap_blocks_tx()) return;

    size_t len = s_tx_queue_len;
    s_tx_busy = true;
    rlr::led::on();
    int n = rlr::radio::transmit(s_tx_queue, len);
    rlr::led::off();
    s_tx_busy = false;
    s_tx_queue_len = 0;

    if (n > 0) {
        s_tx_count++;
        record_tx_airtime(len);
    } else {
        send_byte(CMD_ERROR, ERROR_TXFAILED);
    }
    send_byte(CMD_READY, 0x01);
}

}} // namespace rlr::kiss
