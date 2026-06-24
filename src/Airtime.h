#pragma once
// src/Airtime.h — pure LoRa airtime accounting and RNode status-frame
// payload builders. Header-only and free of any Arduino/RadioLib
// dependency so the native unit tests exercise the same code the
// firmware runs.
//
// Wire formats here are pinned to the host driver
// RNS/Interfaces/RNodeInterface.py (the spec's source of truth, see
// SPEC.md §8.4/§8.5):
//   - ST/LT_ALOCK  : uint16 BE of (percent * 100)          (setSTALock)
//   - CMD_STAT_PHYPRM (12 B): symbol_time_us, symbol_rate,
//                     preamble_syms, preamble_time_ms,
//                     csma_slot_ms, difs_ms — all uint16 BE
//   - CMD_STAT_CHTM  (11 B): airtime_short, airtime_long,
//                     chan_load_short, chan_load_long (uint16 BE of
//                     fraction*10000), rssi+157, noise+157, interference

#include <stdint.h>
#include <stddef.h>
#include <math.h>

namespace rlr { namespace airtime {

inline void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF); }

// ---- LoRa physical-layer timing (Semtech SX1262 datasheet §6.1.4) ----

// Symbol time in milliseconds: Tsym = 2^SF / BW.
inline double symbol_time_ms(uint32_t bw_hz, uint8_t sf) {
    if (bw_hz == 0) return 0.0;
    return ((double)((uint32_t)1 << sf) / (double)bw_hz) * 1000.0;
}

// Symbol rate in symbols/second (baud), clamped to uint16.
inline uint16_t symbol_rate(uint32_t bw_hz, uint8_t sf) {
    double tsym = symbol_time_ms(bw_hz, sf);
    if (tsym <= 0.0) return 0;
    double rate = 1000.0 / tsym;
    if (rate > 65535.0) rate = 65535.0;
    return (uint16_t)(rate + 0.5);
}

// On-air preamble time in ms for `preamble_syms` programmed symbols
// (the air preamble carries an extra fixed 4.25 symbols).
inline double preamble_time_ms(uint32_t bw_hz, uint8_t sf, uint16_t preamble_syms) {
    return ((double)preamble_syms + 4.25) * symbol_time_ms(bw_hz, sf);
}

// Time-on-air (ms) for one LoRa frame whose LoRa payload is
// `payload_len` bytes (the caller has already added any RNode header
// byte into that length). Explicit header, CRC on, and low-data-rate
// optimize auto-enabled when Tsym > 16 ms — matching the RadioLib
// SX1262 configuration this firmware uses.
inline double frame_airtime_ms(uint32_t bw_hz, uint8_t sf, uint8_t cr_denom,
                               uint16_t preamble_syms, size_t payload_len,
                               bool explicit_header = true, bool crc = true) {
    if (bw_hz == 0 || sf == 0) return 0.0;
    double tsym = symbol_time_ms(bw_hz, sf);
    int de = (tsym > 16.0) ? 1 : 0;          // low-data-rate optimize
    int ih = explicit_header ? 0 : 1;        // implicit-header flag
    int crc_on = crc ? 1 : 0;
    double num = (double)(8 * (long)payload_len) - 4 * (long)sf + 28 + 16 * crc_on - 20 * ih;
    double den = 4.0 * ((long)sf - 2 * de);
    double n = ceil(num / den) * (double)cr_denom;
    if (n < 0) n = 0;
    double payload_symbols = 8.0 + n;
    return preamble_time_ms(bw_hz, sf, preamble_syms) + payload_symbols * tsym;
}

// Total airtime (ms, rounded) for an RNode-framed Reticulum packet of
// `payload_len` bytes, accounting for the 1-byte air header and the
// >254-byte split into two independently-preambled LoRa frames. Mirrors
// the TX path in Radio.cpp.
inline uint32_t rnode_packet_airtime_ms(uint32_t bw_hz, uint8_t sf, uint8_t cr_denom,
                                        uint16_t preamble_syms, size_t payload_len) {
    const size_t SINGLE_PAYLOAD = 254;   // payload bytes per frame after the header
    double total;
    if (payload_len <= SINGLE_PAYLOAD) {
        total = frame_airtime_ms(bw_hz, sf, cr_denom, preamble_syms, payload_len + 1);
    } else {
        size_t second = payload_len - SINGLE_PAYLOAD;
        total = frame_airtime_ms(bw_hz, sf, cr_denom, preamble_syms, SINGLE_PAYLOAD + 1)
              + frame_airtime_ms(bw_hz, sf, cr_denom, preamble_syms, second + 1);
    }
    return (uint32_t)(total + 0.5);
}

// ---- Rolling utilisation tracker ----------------------------------
// Binned ring buffer accumulating an occupancy quantity (e.g. our TX
// airtime, or sensed channel-busy time) over a short and a long window
// at once. The clock is injected via now_ms so it is unit-testable
// without a hardware timer. One quantity per instance.
//
// Note: now_ms is the millis() millisecond counter; like the upstream
// firmware this does not specially handle the ~49.7-day uint32 wrap —
// at worst utilisation reads stale for up to one long-window after a
// wrap. The windowing is firmware-private per SPEC.md §8.3/§8.5.
class UtilTracker {
public:
    static const uint32_t SHORT_WINDOW_MS = 15000;     // 15 s
    static const uint32_t LONG_WINDOW_MS  = 3600000;   // 1 hour
    static const int      SHORT_BINS = 15;             // 1 s resolution
    static const int      LONG_BINS  = 60;             // 60 s resolution

    UtilTracker() { reset(); }

    void reset() {
        for (int i = 0; i < SHORT_BINS; i++) short_[i] = 0;
        for (int i = 0; i < LONG_BINS;  i++) long_[i]  = 0;
        short_idx_ = -1;
        long_idx_  = -1;
    }

    void record(uint32_t now_ms, uint32_t amount_ms) {
        roll(now_ms);
        short_[(uint32_t)(now_ms / SHORT_BIN_MS) % SHORT_BINS] += amount_ms;
        long_[(uint32_t)(now_ms / LONG_BIN_MS)  % LONG_BINS]  += amount_ms;
    }

    // Occupancy as a fraction 0..1 of the respective window.
    double short_util(uint32_t now_ms) { roll(now_ms); return (double)sum(short_, SHORT_BINS) / (double)SHORT_WINDOW_MS; }
    double long_util(uint32_t now_ms)  { roll(now_ms); return (double)sum(long_,  LONG_BINS)  / (double)LONG_WINDOW_MS; }

private:
    static const uint32_t SHORT_BIN_MS = SHORT_WINDOW_MS / SHORT_BINS;  // 1000
    static const uint32_t LONG_BIN_MS  = LONG_WINDOW_MS  / LONG_BINS;   // 60000

    uint32_t short_[SHORT_BINS];
    uint32_t long_[LONG_BINS];
    int64_t  short_idx_;   // absolute index of newest touched short bin
    int64_t  long_idx_;

    void roll(uint32_t now_ms) {
        clear_stale((int64_t)(now_ms / SHORT_BIN_MS), short_idx_, short_, SHORT_BINS);
        clear_stale((int64_t)(now_ms / LONG_BIN_MS),  long_idx_,  long_,  LONG_BINS);
    }
    static void clear_stale(int64_t cur, int64_t& last, uint32_t* bins, int n) {
        if (last < 0) { last = cur; return; }
        if (cur <= last) return;
        int64_t steps = cur - last;
        if (steps >= n) {
            for (int i = 0; i < n; i++) bins[i] = 0;
        } else {
            for (int64_t s = 1; s <= steps; s++) bins[(last + s) % n] = 0;
        }
        last = cur;
    }
    static uint32_t sum(const uint32_t* bins, int n) {
        uint32_t s = 0;
        for (int i = 0; i < n; i++) s += bins[i];
        return s;
    }
};

// ---- KISS status-frame payload builders ---------------------------

// ST/LT_ALOCK: uint16 of (percent * 100), e.g. 30.00% -> 3000.
inline uint16_t encode_alock(double percent) {
    double v = percent * 100.0;
    if (v < 0) v = 0;
    if (v > 65535.0) v = 65535.0;
    return (uint16_t)(v + 0.5);
}
inline double decode_alock(uint16_t raw) { return (double)raw / 100.0; }

// CMD_STAT_PHYPRM payload (12 bytes).
inline void build_phyprm(uint8_t out[12], uint32_t bw_hz, uint8_t sf,
                         uint16_t preamble_syms,
                         uint16_t csma_slot_ms, uint16_t difs_ms) {
    double tsym = symbol_time_ms(bw_hz, sf);
    double lst_us = tsym * 1000.0;                       // host divides by 1000 -> ms
    if (lst_us > 65535.0) lst_us = 65535.0;
    double prt = preamble_time_ms(bw_hz, sf, preamble_syms);
    if (prt > 65535.0) prt = 65535.0;
    put16(out + 0,  (uint16_t)(lst_us + 0.5));
    put16(out + 2,  symbol_rate(bw_hz, sf));
    put16(out + 4,  preamble_syms);
    put16(out + 6,  (uint16_t)(prt + 0.5));
    put16(out + 8,  csma_slot_ms);
    put16(out + 10, difs_ms);
}

// CMD_STAT_CHTM payload (11 bytes). Airtime/load args are fractions 0..1.
inline void build_chtm(uint8_t out[11],
                       double airtime_short, double airtime_long,
                       double load_short, double load_long,
                       int current_rssi_dbm, int noise_floor_dbm,
                       uint8_t interference) {
    // host: value/100.0 -> percentage, so fraction*100*100 == fraction*10000.
    auto pct = [](double frac) -> uint16_t {
        double v = frac * 10000.0;
        if (v < 0) v = 0;
        if (v > 65535.0) v = 65535.0;
        return (uint16_t)(v + 0.5);
    };
    put16(out + 0, pct(airtime_short));
    put16(out + 2, pct(airtime_long));
    put16(out + 4, pct(load_short));
    put16(out + 6, pct(load_long));
    out[8]  = (uint8_t)(current_rssi_dbm + 157);
    out[9]  = (uint8_t)(noise_floor_dbm + 157);
    out[10] = interference;
}

// CMD_STAT_CSMA payload (3 bytes): contention-window band / min / max.
inline void build_csma(uint8_t out[3], uint8_t cw_band, uint8_t cw_min, uint8_t cw_max) {
    out[0] = cw_band;
    out[1] = cw_min;
    out[2] = cw_max;
}

}} // namespace rlr::airtime
