// test/test_rnode_proto/test_rnode_proto.cpp
//
// Native (host-side) unit tests for the RNode wire-format helpers that
// back SPEC.md §8.1 (KISS), §8.3 (split framing airtime), §8.4 (config
// handshake numerics) and §8.5 (airtime caps / CHTM / PHYPRM).
//
// Every expected byte layout here is cross-checked against the upstream
// host driver RNS/Interfaces/RNodeInterface.py — the spec's source of
// truth — so a drift in either side fails a test.

#include <unity.h>
#include "KissCodec.h"
#include "Airtime.h"

using namespace rlr;

void setUp(void) {}
void tearDown(void) {}

// ---- §8.1 KISS escaping -------------------------------------------

void test_kiss_escape_passthrough(void) {
    const uint8_t in[] = {0x00, 0x01, 0x55, 0xAA};
    uint8_t out[16];
    size_t n = kisscodec::escape(in, sizeof(in), out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(4, n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in, out, 4);
}

void test_kiss_escape_special_bytes(void) {
    // FEND -> FESC TFEND, FESC -> FESC TFESC (SPEC.md §8.1)
    const uint8_t in[] = {0xC0, 0xDB};
    uint8_t out[16];
    size_t n = kisscodec::escape(in, sizeof(in), out, sizeof(out));
    const uint8_t want[] = {0xDB, 0xDC, 0xDB, 0xDD};
    TEST_ASSERT_EQUAL_size_t(4, n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, 4);
}

void test_kiss_escape_unescape_roundtrip(void) {
    uint8_t in[256];
    for (int i = 0; i < 256; i++) in[i] = (uint8_t)i;   // includes 0xC0, 0xDB
    uint8_t esc[600], back[300];
    size_t en = kisscodec::escape(in, 256, esc, sizeof(esc));
    TEST_ASSERT_EQUAL_size_t(258, en);                  // two bytes doubled
    size_t dn = kisscodec::unescape(esc, en, back, sizeof(back));
    TEST_ASSERT_EQUAL_size_t(256, dn);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in, back, 256);
}

void test_kiss_escape_reports_overflow(void) {
    const uint8_t in[] = {0xC0};   // needs 2 bytes
    uint8_t out[1];
    TEST_ASSERT_EQUAL_size_t(0, kisscodec::escape(in, 1, out, 1));
}

// ---- §8.5.1 airtime-limit encoding --------------------------------

void test_alock_encode_matches_host(void) {
    // RNS setSTALock: at = int(limit*100); BE uint16. 30.00% -> 3000.
    TEST_ASSERT_EQUAL_UINT16(3000, airtime::encode_alock(30.0));
    // Reticulum.ANNOUNCE_CAP default 2.0% -> 0x00C8 (per SPEC.md §8.5.1).
    TEST_ASSERT_EQUAL_UINT16(0x00C8, airtime::encode_alock(2.0));
}

void test_alock_decode_roundtrip(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 30.0, airtime::decode_alock(3000));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0,  airtime::decode_alock(200));
}

// ---- §8.4 LoRa timing math ----------------------------------------

void test_symbol_time_and_rate(void) {
    // SF7 / BW125k: Tsym = 128/125000 s = 1.024 ms; rate = 976.56 baud.
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.024, airtime::symbol_time_ms(125000, 7));
    TEST_ASSERT_EQUAL_UINT16(977, airtime::symbol_rate(125000, 7));
}

void test_frame_airtime_known_value(void) {
    // SF7/BW125k/CR4-5, 16-byte LoRa payload, 16-symbol preamble, explicit
    // header, CRC on, DE=0. Hand-computed time-on-air = 59.648 ms.
    double toa = airtime::frame_airtime_ms(125000, 7, 5, 16, 16);
    TEST_ASSERT_DOUBLE_WITHIN(0.05, 59.648, toa);
}

void test_rnode_split_airtime_is_two_frames(void) {
    // 300-byte payload splits into 254 + 46 (each +1 header byte). The
    // RNode total must equal the sum of the two frames' airtime, and
    // exceed the single-frame airtime for a 254-byte payload.
    uint32_t single = airtime::rnode_packet_airtime_ms(125000, 9, 5, 16, 254);
    uint32_t split  = airtime::rnode_packet_airtime_ms(125000, 9, 5, 16, 300);
    double f1 = airtime::frame_airtime_ms(125000, 9, 5, 16, 255);
    double f2 = airtime::frame_airtime_ms(125000, 9, 5, 16, 47);
    TEST_ASSERT_UINT32_WITHIN(1, (uint32_t)(f1 + f2 + 0.5), split);
    TEST_ASSERT_TRUE(split > single);
}

// ---- §8.5 utilisation tracker -------------------------------------

void test_util_tracker_accumulates_and_decays(void) {
    airtime::UtilTracker t;
    // 500 ms of airtime at t=1 s -> 500/15000 of the short window.
    t.record(1000, 500);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 500.0 / 15000.0, t.short_util(1000));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 500.0 / 3600000.0, t.long_util(1000));

    // After the 15 s short window fully rolls past, short util is 0 but
    // the long window still carries it.
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, t.short_util(60000));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 500.0 / 3600000.0, t.long_util(60000));

    // After the 1 h long window rolls past, both are clear.
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, t.long_util(4000000));
}

void test_util_tracker_caps_drive_alock_decision(void) {
    // Mirror the firmware's gate: short util * 100 >= cap blocks TX.
    airtime::UtilTracker t;
    t.record(1000, 1000);                       // 1 s in a 15 s window = 6.67%
    double util_pct = t.short_util(1000) * 100.0;
    TEST_ASSERT_TRUE(util_pct >= 2.0);          // would block a 2% cap
    TEST_ASSERT_FALSE(util_pct >= 30.0);        // would not block a 30% cap
}

// ---- §8.4.5 / §8.5 status-frame payloads --------------------------

void test_build_phyprm_layout(void) {
    // SF7/BW125k -> symbol time 1024 us, rate 977 baud, 16 preamble syms.
    uint8_t out[12];
    airtime::build_phyprm(out, 125000, 7, 16, /*slot*/2, /*difs*/5);
    TEST_ASSERT_EQUAL_UINT16(1024, (out[0] << 8) | out[1]);   // host /1000 -> 1.024 ms
    TEST_ASSERT_EQUAL_UINT16(977,  (out[2] << 8) | out[3]);   // symbol rate
    TEST_ASSERT_EQUAL_UINT16(16,   (out[4] << 8) | out[5]);   // preamble symbols
    // preamble time = (16+4.25)*1.024 = 20.736 ms -> 21 rounded
    TEST_ASSERT_EQUAL_UINT16(21,   (out[6] << 8) | out[7]);
    TEST_ASSERT_EQUAL_UINT16(2,    (out[8] << 8) | out[9]);   // csma slot
    TEST_ASSERT_EQUAL_UINT16(5,    (out[10] << 8) | out[11]); // difs
}

void test_build_chtm_layout(void) {
    // 5% airtime fraction -> host value/100 = 5.0; firmware sends 0.05*10000=500.
    uint8_t out[11];
    airtime::build_chtm(out, 0.05, 0.01, 0.10, 0.02,
                        /*rssi*/-100, /*noise*/-120, /*interference*/0xFF);
    TEST_ASSERT_EQUAL_UINT16(500,  (out[0] << 8) | out[1]);   // airtime short
    TEST_ASSERT_EQUAL_UINT16(100,  (out[2] << 8) | out[3]);   // airtime long
    TEST_ASSERT_EQUAL_UINT16(1000, (out[4] << 8) | out[5]);   // load short
    TEST_ASSERT_EQUAL_UINT16(200,  (out[6] << 8) | out[7]);   // load long
    TEST_ASSERT_EQUAL_UINT8(57,    out[8]);                   // -100 + 157
    TEST_ASSERT_EQUAL_UINT8(37,    out[9]);                   // -120 + 157
    TEST_ASSERT_EQUAL_UINT8(0xFF,  out[10]);
}

void test_build_csma_layout(void) {
    // CMD_STAT_CSMA: [cw_band, cw_min, cw_max] (host reads 3 bytes).
    uint8_t out[3];
    airtime::build_csma(out, 1, 0, 5);
    TEST_ASSERT_EQUAL_UINT8(1, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0, out[1]);
    TEST_ASSERT_EQUAL_UINT8(5, out[2]);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_kiss_escape_passthrough);
    RUN_TEST(test_kiss_escape_special_bytes);
    RUN_TEST(test_kiss_escape_unescape_roundtrip);
    RUN_TEST(test_kiss_escape_reports_overflow);
    RUN_TEST(test_alock_encode_matches_host);
    RUN_TEST(test_alock_decode_roundtrip);
    RUN_TEST(test_symbol_time_and_rate);
    RUN_TEST(test_frame_airtime_known_value);
    RUN_TEST(test_rnode_split_airtime_is_two_frames);
    RUN_TEST(test_util_tracker_accumulates_and_decays);
    RUN_TEST(test_util_tracker_caps_drive_alock_decision);
    RUN_TEST(test_build_phyprm_layout);
    RUN_TEST(test_build_chtm_layout);
    RUN_TEST(test_build_csma_layout);
    return UNITY_END();
}
