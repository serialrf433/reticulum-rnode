#pragma once
// src/Radio.h — non-blocking SX1262 radio (RadioLib), RNode air framing.
//
// The radio core NEVER busy-waits on airtime. At slow data rates a single
// LoRa frame is hundreds of ms to seconds; blocking that long starves the
// nRF52 SoftDevice and drops the BLE link (supervision timeout — observed
// as Sideband connection flapping). So, following the sibling project's
// proven model:
//
//   * TX uses RadioLib's async startTransmit()/finishTransmit(),
//   * the SX1262 DIO1 line fires an ISR that ONLY sets a volatile flag,
//   * poll() — called every loop() — does the SPI work outside interrupt
//     context, advances a small TX/RX state machine, and re-arms RX.
//
// RNode specifics live here too: the 1-byte air header (random seq nibble +
// FLAG_SPLIT) and the >254-byte split into two LoRa frames, plus RX-side
// reassembly. Received, fully-reassembled Reticulum payloads are delivered
// via the RxCallback (invoked from poll(), not from interrupt context).

#include "Config.h"
#include <stdint.h>
#include <stddef.h>

namespace rlr { namespace radio {

// Invoked from poll() when a complete (reassembled) Reticulum packet arrives.
// buf/len are valid only for the duration of the call; rssi in dBm, snr in dB.
typedef void (*RxCallback)(const uint8_t* buf, size_t len, float rssi, float snr);

// Assert VEXT/SPI pins and construct the RadioLib objects. Does not talk to
// the chip yet. Returns true on success.
bool init_hardware();

// Apply freq/BW/SF/CR/TXP, sync word, TCXO, RF-switch config and leave the
// radio in standby. Returns true on success.
bool begin(const Config& cfg);

// Register the receive callback (call before start_rx()).
void set_rx_callback(RxCallback cb);

// Enter continuous RX. Call after begin(). Returns true on success.
bool start_rx();

// Query whether the radio passed begin().
bool online();

// Put the radio into standby. Reversible via begin()/start_rx().
void stop();

// Drive the TX/RX state machine. Call every loop() iteration. Cheap and
// non-blocking when nothing is pending. Delivers RX packets via the callback.
void poll();

// Queue a packet (full Reticulum payload, <= 508 bytes) for transmission.
// Non-blocking: the packet is framed (header + optional split) and aired via
// CSMA over subsequent poll() calls. Returns false if the radio is busy with
// another TX, an RX event is pending, the radio is offline, or len is invalid.
bool enqueue_tx(const uint8_t* buf, size_t len);

// True while a TX is pending CSMA or in flight (enqueue_tx will refuse).
bool tx_busy();

// Returns true exactly once after a queued packet finishes transmitting,
// then clears. Used by the host layer to send CMD_READY / account airtime.
bool tx_just_completed();

// Full payload length of the most recently completed transmission.
size_t last_tx_len();

// Last-received signal quality (updated before each RX callback).
float last_rssi();
float last_snr();

// Instantaneous channel RSSI in dBm (valid while in RX).
float read_rssi();

// True if the channel RSSI is below the CSMA threshold (clear to transmit).
bool channel_clear();

// CSMA contention-window descriptor for CMD_STAT_CSMA (diagnostic).
void csma_params(uint8_t& cw_band, uint8_t& cw_min, uint8_t& cw_max);

}} // namespace rlr::radio
