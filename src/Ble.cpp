// src/Ble.cpp — BLE module with Nordic UART Service (NUS).
//
// Matches the official RNode firmware's BLE implementation:
//   - Device name: "RNode XXXX" (MD5 of MAC + EEPROM signature byte)
//   - BLEUart with 6144-byte RX FIFO, buffered TXD, frame-level flush
//   - Bandwidth MAX, 515-byte MTU, Data Length Extension
//   - Just Works bonding (phone shows confirmation, tap accept)

#include "Ble.h"

#if HAS_BLE

#include <Arduino.h>
#include <bluefruit.h>
#include "Eeprom.h"
#include "Kiss.h"  // ADDR_BLE_PIN, BLE_PIN_LEN

namespace rlr { namespace ble {

// ---- Minimal MD5 (RFC 1321) for BLE name hash -----------------------

namespace {

static const uint32_t md5_s[] = {
    7,12,17,22, 7,12,17,22, 7,12,17,22, 7,12,17,22,
    5, 9,14,20, 5, 9,14,20, 5, 9,14,20, 5, 9,14,20,
    4,11,16,23, 4,11,16,23, 4,11,16,23, 4,11,16,23,
    6,10,15,21, 6,10,15,21, 6,10,15,21, 6,10,15,21,
};

static const uint32_t md5_k[] = {
    0xd76aa478,0xe8c7b756,0x242070db,0xc1bdceee,
    0xf57c0faf,0x4787c62a,0xa8304613,0xfd469501,
    0x698098d8,0x8b44f7af,0xffff5bb1,0x895cd7be,
    0x6b901122,0xfd987193,0xa679438e,0x49b40821,
    0xf61e2562,0xc040b340,0x265e5a51,0xe9b6c7aa,
    0xd62f105d,0x02441453,0xd8a1e681,0xe7d3fbc8,
    0x21e1cde6,0xc33707d6,0xf4d50d87,0x455a14ed,
    0xa9e3e905,0xfcefa3f8,0x676f02d9,0x8d2a4c8a,
    0xfffa3942,0x8771f681,0x6d9d6122,0xfde5380c,
    0xa4beea44,0x4bdecfa9,0xf6bb4b60,0xbebfbc70,
    0x289b7ec6,0xeaa127fa,0xd4ef3085,0x04881d05,
    0xd9d4d039,0xe6db99e5,0x1fa27cf8,0xc4ac5665,
    0xf4292244,0x432aff97,0xab9423a7,0xfc93a039,
    0x655b59c3,0x8f0ccc92,0xffeff47d,0x85845dd1,
    0x6fa87e4f,0xfe2ce6e0,0xa3014314,0x4e0811a1,
    0xf7537e82,0xbd3af235,0x2ad7d2bb,0xeb86d391,
};

static inline uint32_t _rotl(uint32_t x, uint32_t n) {
    return (x << n) | (x >> (32 - n));
}

static void md5_hash(const uint8_t* data, size_t len, uint8_t out[16]) {
    uint32_t a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
    size_t padded_len = ((len + 8) / 64 + 1) * 64;
    uint8_t* msg = (uint8_t*)calloc(padded_len, 1);
    memcpy(msg, data, len);
    msg[len] = 0x80;
    uint64_t bit_len = (uint64_t)len * 8;
    memcpy(msg + padded_len - 8, &bit_len, 8);

    for (size_t offset = 0; offset < padded_len; offset += 64) {
        uint32_t* M = (uint32_t*)(msg + offset);
        uint32_t A = a0, B = b0, C = c0, D = d0;
        for (uint32_t i = 0; i < 64; i++) {
            uint32_t F, g;
            if (i < 16)      { F = (B & C) | (~B & D); g = i; }
            else if (i < 32) { F = (D & B) | (~D & C); g = (5*i+1) % 16; }
            else if (i < 48) { F = B ^ C ^ D;          g = (3*i+5) % 16; }
            else              { F = C ^ (B | ~D);       g = (7*i) % 16; }
            F = F + A + md5_k[i] + M[g];
            A = D; D = C; C = B; B = B + _rotl(F, md5_s[i]);
        }
        a0 += A; b0 += B; c0 += C; d0 += D;
    }
    free(msg);
    memcpy(out +  0, &a0, 4);
    memcpy(out +  4, &b0, 4);
    memcpy(out +  8, &c0, 4);
    memcpy(out + 12, &d0, 4);
}

} // anonymous namespace

// ---- BLE state ------------------------------------------------------

static constexpr uint16_t BLE_RX_BUF = 6144;
static BLEUart s_ble_uart(BLE_RX_BUF);
static BLEDis  s_ble_dis;
static BLEBas  s_ble_bas;
static bool    s_connected = false;
static char    s_devname[12];  // "RNode XXXX\0"

// ---- Callbacks ------------------------------------------------------

static void _connect_callback(uint16_t conn_handle) {
    // Don't set s_connected here — too early. The central hasn't
    // subscribed to NUS TX notifications yet, so writes would be lost.
    // We set s_connected in _secured_callback (after bonding) or
    // _notify_callback (when central subscribes to TX).
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    if (conn) {
        conn->requestPHY(BLE_GAP_PHY_2MBPS);
        conn->requestMtuExchange(515);
        conn->requestDataLengthUpdate();
    }
    Serial.println("BLE: central connected (waiting for bond + notify)");
}

static void _disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    s_connected = false;
    Serial.print("BLE: disconnected, reason=0x");
    Serial.println(reason, HEX);
}

// Fired when security/bonding completes (Just Works or MITM).
// In the official RNode firmware, BT_STATE_CONNECTED is set here.
static void _secured_callback(uint16_t conn_handle) {
    (void)conn_handle;
    Serial.println("BLE: bonded/secured");
    // Don't set s_connected yet — wait for notify subscription
}

// Fired when the central subscribes/unsubscribes to NUS TX notifications.
// This is the safest point to enable the transport — the central is
// ready to receive KISS responses.
static void _notify_callback(uint16_t conn_hdl, bool enabled) {
    (void)conn_hdl;
    if (enabled) {
        s_connected = true;
        Serial.println("BLE: TX notify enabled — transport active");
    } else {
        s_connected = false;
        Serial.println("BLE: TX notify disabled — transport inactive");
    }
}

// ---- Public API -----------------------------------------------------

void init(const char* device_name) {
    (void)device_name;
    Serial.println("BLE: initializing...");

    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.setTxPower(8);

    // Compute "RNode XXXX" name
    const ble_gap_addr_t gap_addr = Bluefruit.getAddr();
    uint8_t hash_input[7];
    memcpy(hash_input, gap_addr.addr, 6);
    hash_input[6] = rlr::eeprom::read_byte(0x1B);  // ADDR_SIGNATURE first byte

    uint8_t md5[16];
    md5_hash(hash_input, 7, md5);
    sprintf(s_devname, "RNode %02X%02X", md5[14], md5[15]);
    Bluefruit.setName(s_devname);

    // Connection callbacks
    Bluefruit.Periph.setConnectCallback(_connect_callback);
    Bluefruit.Periph.setDisconnectCallback(_disconnect_callback);
    Bluefruit.Periph.setConnInterval(6, 12);  // 7.5 - 15 ms

    // Security: PIN-based pairing (MITM via DisplayOnly IO caps).
    // PIN is stored at EEPROM[ADDR_BLE_PIN]; defaults to "123456" if
    // unset (all 0xFF). The user can change it via CMD_BLE_PIN over
    // KISS or via the webflasher; takes effect on next reboot.
    char ble_pin[rlr::kiss::BLE_PIN_LEN + 1];
    rlr::eeprom::read(rlr::kiss::ADDR_BLE_PIN, (uint8_t*)ble_pin, rlr::kiss::BLE_PIN_LEN);
    bool pin_valid = true;
    for (size_t i = 0; i < rlr::kiss::BLE_PIN_LEN; i++) {
        if (ble_pin[i] < '0' || ble_pin[i] > '9') { pin_valid = false; break; }
    }
    if (!pin_valid) {
        memcpy(ble_pin, "123456", rlr::kiss::BLE_PIN_LEN);
    }
    ble_pin[rlr::kiss::BLE_PIN_LEN] = '\0';
    Bluefruit.Security.setPIN(ble_pin);
    Bluefruit.Security.setSecuredCallback(_secured_callback);
    Serial.print("BLE: pairing PIN = "); Serial.println(ble_pin);

    // Device Information Service
    s_ble_dis.setManufacturer(BOARD_MANUFACTURER);
    s_ble_dis.setModel(BOARD_NAME);
    s_ble_dis.begin();

    // Battery Service
    s_ble_bas.begin();

    // Nordic UART Service — buffered TXD for frame-level flushing.
    // REQUIRE MITM-authenticated encryption on the NUS read/write/notify
    // permissions so the central MUST pair with the PIN set above before
    // it can use the service. Without this, setPIN() alone doesn't force
    // a PIN prompt — Android will happily "Just Works" pair silently
    // because NUS chars default to no security requirement.
    s_ble_uart.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
    s_ble_uart.bufferTXD(true);
    s_ble_uart.setNotifyCallback(_notify_callback);
    s_ble_uart.begin();

    // Advertising: NUS UUID in adv packet, name in scan response
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(s_ble_uart);
    Bluefruit.ScanResponse.addName();

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);

    Serial.print("BLE: advertising as '");
    Serial.print(s_devname);
    Serial.println("'");
}

bool connected() {
    return s_connected;
}

const char* device_name() {
    return s_devname;
}

int available() {
    return s_ble_uart.available();
}

int read() {
    return s_ble_uart.read();
}

size_t write(const uint8_t* buf, size_t len) {
    if (!s_connected) return 0;
    return s_ble_uart.write(buf, len);
}

size_t write(uint8_t b) {
    if (!s_connected) return 0;
    return s_ble_uart.write(b);
}

void flush() {
    if (s_connected) {
        s_ble_uart.flushTXD();
    }
}

}} // namespace rlr::ble

#else // !HAS_BLE

namespace rlr { namespace ble {
void init(const char*) {}
bool connected() { return false; }
const char* device_name() { return ""; }
int available() { return 0; }
int read() { return -1; }
size_t write(const uint8_t*, size_t) { return 0; }
size_t write(uint8_t) { return 0; }
void flush() {}
}} // namespace rlr::ble

#endif // HAS_BLE
