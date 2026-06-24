// =====================================================================
//  reticulum-rnode / src/main.cpp
//  ------------------------------------------------------------------
//  RNode firmware with KISS interface for nRF52840 + SX1262 boards.
//  Serial-to-LoRa bridge — the host runs Reticulum via RNodeInterface,
//  this firmware handles the radio.
// =====================================================================

#include <Arduino.h>
#include "Config.h"
#include "Radio.h"
#include "Led.h"
#include "Kiss.h"
#include "Storage.h"
#include "Eeprom.h"
#include "Battery.h"
#include "Ble.h"

#ifndef RLR_VERSION
  #define RLR_VERSION "0.1.0-dev"
#endif

// Delivered from rlr::radio::poll() when a complete Reticulum packet arrives.
static void on_radio_rx(const uint8_t* buf, size_t len, float rssi, float snr) {
    rlr::led::on();
    rlr::kiss::send_rx_packet(buf, len, rssi, snr);
    rlr::led::off();
}

void setup() {
    Serial.begin(115200);
    // Give USB CDC time to enumerate
    uint32_t wait_start = millis();
    while (!Serial && (millis() - wait_start) < 4000) {
        delay(10);
    }

    Serial.println();
    Serial.println("=====================================================");
    Serial.print("  reticulum-rnode ");
    Serial.println(RLR_VERSION);
    Serial.print("  Board: ");
    Serial.println(BOARD_NAME);
    Serial.print("  Radio: ");
    Serial.print(RADIO_CHIP);
    Serial.print(" (");
    Serial.print(RADIO_MODULE);
    Serial.println(")");
    Serial.println("  Mode: KISS RNode");
    Serial.println("=====================================================");

    rlr::led::init();

    // Initialize storage, EEPROM, and battery
    rlr::storage::init();
    rlr::eeprom::init();
    rlr::battery::init();

    // Initialize BLE with Nordic UART Service
    rlr::ble::init(BOARD_NAME);

    // Initialize radio hardware (VEXT, SPI)
    if (!rlr::radio::init_hardware()) {
        Serial.println("Setup: radio::init_hardware() failed");
    }
    rlr::radio::set_rx_callback(on_radio_rx);

    // Initialize KISS processor
    rlr::kiss::init();

    Serial.println("Setup complete — waiting for host KISS commands.");
}

void loop() {
    // Process incoming KISS frames from host
    rlr::kiss::tick();

    // Drive the non-blocking radio state machine (services TX completion and
    // delivers RX packets via on_radio_rx) — never busy-waits on airtime, so
    // BLE keeps being serviced during transmissions.
    rlr::radio::poll();

    // Finalize completed TX (CMD_READY + airtime) and start any queued packet.
    rlr::kiss::tx_service();

    // Heartbeat LED
    static rlr::Config s_minimal_cfg = { 0, 0, 0, 0, 0, rlr::CONFIG_FLAG_HEARTBEAT };
    rlr::led::heartbeat_tick(s_minimal_cfg);
}
