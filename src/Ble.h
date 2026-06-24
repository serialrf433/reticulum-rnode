#pragma once
// src/Ble.h — BLE module with Nordic UART Service (NUS).
//
// Exposes BLE UART as a Stream so Sideband (and other Reticulum apps)
// can connect to the RNode wirelessly. The KISS protocol runs over
// BLE NUS the same way it runs over USB Serial.
//
// Only compiled on boards with HAS_BLE=1.

#include <stdint.h>
#include <stddef.h>

namespace rlr { namespace ble {

// Initialize Bluefruit BLE with NUS, Device Info Service, and
// start advertising. Call from setup().
void init(const char* device_name);

// Returns true if a BLE central is connected and NUS notify is enabled.
bool connected();

// Returns the advertised BLE device name ("RNode XXXX"). Returns an
// empty string on builds without BLE. Valid after init().
const char* device_name();

// Returns the number of bytes available to read from BLE UART.
int available();

// Read one byte from BLE UART. Returns -1 if none available.
int read();

// Write bytes to BLE UART (sends as NUS notifications).
size_t write(const uint8_t* buf, size_t len);
size_t write(uint8_t b);

// Flush buffered TXD data as a BLE notification. Call after
// writing each complete KISS frame (at the closing FEND).
void flush();

}} // namespace rlr::ble
