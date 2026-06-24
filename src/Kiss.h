#pragma once
// src/Kiss.h — KISS frame encoder/decoder for RNode serial protocol.
//
// Reads KISS frames from Serial, dispatches commands, and provides
// helpers to send KISS-framed responses back to the host.

#include <stdint.h>
#include <stddef.h>

namespace rlr { namespace kiss {

// KISS framing constants
constexpr uint8_t FEND  = 0xC0;
constexpr uint8_t FESC  = 0xDB;
constexpr uint8_t TFEND = 0xDC;
constexpr uint8_t TFESC = 0xDD;

// RNode command bytes
constexpr uint8_t CMD_DATA        = 0x00;
constexpr uint8_t CMD_FREQUENCY   = 0x01;
constexpr uint8_t CMD_BANDWIDTH   = 0x02;
constexpr uint8_t CMD_TXPOWER     = 0x03;
constexpr uint8_t CMD_SF          = 0x04;
constexpr uint8_t CMD_CR          = 0x05;
constexpr uint8_t CMD_RADIO_STATE = 0x06;
constexpr uint8_t CMD_RADIO_LOCK  = 0x07;
constexpr uint8_t CMD_DETECT      = 0x08;
constexpr uint8_t CMD_IMPLICIT    = 0x09;
constexpr uint8_t CMD_LEAVE       = 0x0A;
constexpr uint8_t CMD_ST_ALOCK    = 0x0B;
constexpr uint8_t CMD_LT_ALOCK    = 0x0C;
constexpr uint8_t CMD_PROMISC     = 0x0E;
constexpr uint8_t CMD_READY       = 0x0F;

constexpr uint8_t CMD_STAT_RX     = 0x21;
constexpr uint8_t CMD_STAT_TX     = 0x22;
constexpr uint8_t CMD_STAT_RSSI   = 0x23;
constexpr uint8_t CMD_STAT_SNR    = 0x24;
constexpr uint8_t CMD_STAT_CHTM   = 0x25;
constexpr uint8_t CMD_STAT_PHYPRM = 0x26;
constexpr uint8_t CMD_STAT_BAT    = 0x27;
constexpr uint8_t CMD_STAT_CSMA   = 0x28;
constexpr uint8_t CMD_STAT_TEMP   = 0x29;

constexpr uint8_t CMD_BLINK       = 0x30;
constexpr uint8_t CMD_RANDOM      = 0x40;
constexpr uint8_t CMD_BOARD       = 0x47;
constexpr uint8_t CMD_PLATFORM    = 0x48;
constexpr uint8_t CMD_MCU         = 0x49;
constexpr uint8_t CMD_FW_VERSION  = 0x50;
constexpr uint8_t CMD_ROM_READ    = 0x51;
constexpr uint8_t CMD_ROM_WRITE   = 0x52;
constexpr uint8_t CMD_CONF_SAVE   = 0x53;
constexpr uint8_t CMD_CONF_DELETE = 0x54;
constexpr uint8_t CMD_RESET       = 0x55;
// CMD_RESET payloads: 0xF8 = plain system reset; 0xDF = reset into the
// Adafruit serial-DFU bootloader (reliable replacement for the flaky
// 1200-baud USB touch on SoftDevice/BLE boards).
constexpr uint8_t RESET_TO_BOOTLOADER = 0xDF;
constexpr uint8_t CMD_DEV_HASH    = 0x56;
constexpr uint8_t CMD_DEV_SIG     = 0x57;
constexpr uint8_t CMD_FW_HASH     = 0x58;
constexpr uint8_t CMD_UNLOCK_ROM  = 0x59;
constexpr uint8_t CMD_HASHES      = 0x60;
constexpr uint8_t CMD_FW_UPD      = 0x61;
constexpr uint8_t CMD_CFG_READ    = 0x6D;
constexpr uint8_t CMD_BLE_PIN     = 0x70;  // Read/write BLE pairing PIN
constexpr uint8_t CMD_BLE_NAME    = 0x71;  // Read advertised BLE device name

constexpr uint8_t CMD_ERROR       = 0x90;

// EEPROM layout additions (beyond the standard RNode map)
constexpr uint16_t ADDR_BLE_PIN   = 0xC0;  // 6 ASCII digits, 0xFF = unset
constexpr size_t   BLE_PIN_LEN    = 6;

// Detect handshake
constexpr uint8_t DETECT_REQ  = 0x73;
constexpr uint8_t DETECT_RESP = 0x46;

// Platform/MCU identifiers
constexpr uint8_t PLATFORM_NRF52 = 0x70;
constexpr uint8_t MCU_NRF52      = 0x71;

// Error codes
constexpr uint8_t ERROR_INITRADIO    = 0x01;
constexpr uint8_t ERROR_TXFAILED     = 0x02;
constexpr uint8_t ERROR_EEPROM_LOCK  = 0x03;
constexpr uint8_t ERROR_QUEUE_FULL   = 0x04;
constexpr uint8_t ERROR_MEMORY_LOW   = 0x05;
constexpr uint8_t ERROR_MODEM_TIMEOUT = 0x06;

// RSSI offset (RNode convention: reported = actual + 157)
constexpr int RSSI_OFFSET = 157;

// Initialize the KISS processor.
void init();

// Process incoming KISS bytes from Serial and dispatch commands.
// Call from loop().
void tick();

// Send a KISS-framed response to the host.
void send_frame(uint8_t cmd, const uint8_t* data, size_t len);

// Convenience: send a single-byte response.
void send_byte(uint8_t cmd, uint8_t value);

// Send an RX packet to the host (RSSI + SNR + DATA frames).
void send_rx_packet(const uint8_t* data, size_t len, float rssi, float snr);

// Drain queued TX packet (if any). Call from loop().
// Finalize a completed async radio TX and start any queued packet.
// Call from loop() after rlr::radio::poll().
void tx_service();

}} // namespace rlr::kiss
