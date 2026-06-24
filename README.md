> [!NOTE]
> **🙋 Seeking a co-maintainer / partner.** I'm stepping back from active development of this repo and looking for someone to help maintain this project (or take it over). Bug reports with a clear repro are welcome; feature requests are PR-welcome but not promised. Interested in partnering?**.

# reticulum-rnode

RNode firmware with KISS serial interface for nRF52840 + SX1262 boards. Acts as a USB-connected LoRa radio modem for [Reticulum](https://reticulum.network/)'s `RNodeInterface`.

The host runs the Reticulum stack (via `rnsd`, Sideband, or NomadNet); this firmware handles the radio.

## Supported Boards

| Board | Radio Module | Status |
|-------|-------------|--------|
| Faketec (Nice!Nano + E22-900M30S) | SX1262 + ext PA | Tested |
| RAK4631 (WisBlock Core) | SX1262 integrated | Untested |
| Seeed XIAO nRF52840 Kit | Wio-SX1262 | Untested |
| Heltec Mesh Node T114 | SX1262 integrated | Untested |
| RAK3401 1-Watt | SX1262 + 1W PA | Untested |
| LilyGO T-Echo | SX1262 integrated | Untested — pins from Meshtastic, not bench-validated |

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
# Build for a specific board
pio run -e Faketec
pio run -e RAK4631
pio run -e XIAO_nRF52840
pio run -e Heltec_T114
pio run -e RAK3401
pio run -e T-Echo

# Flash via nrfutil (USB bootloader)
pio run -e Faketec -t upload --upload-port COMxx

# Serial monitor
pio device monitor -e Faketec --port COMxx
```

The build produces `firmware.hex`, `firmware.zip` (nrfutil DFU package), and `firmware.uf2` (for boards with UF2 bootloader like XIAO).

## Usage with Reticulum

1. Flash the firmware to your board.
2. Connect via USB. The board appears as a USB CDC serial port at 115200 baud.
3. Configure Reticulum to use `RNodeInterface`:

```ini
# ~/.reticulum/config

[interfaces]
  [[RNode LoRa]]
    type = RNodeInterface
    port = /dev/ttyACM0       # or COMxx on Windows
    frequency = 915000000
    bandwidth = 125000
    spreading_factor = 10
    coding_rate = 5
    txpower = 22
```

4. Start `rnsd` or your Reticulum application. The host will auto-detect the RNode and configure it via KISS commands.

## Bluetooth (BLE) Pairing

The firmware advertises itself over BLE as `RNode XXXX` (the suffix is
derived from the device's MAC address). When pairing from a phone, Android
(and iOS) will show a **PIN entry prompt** — the default PIN is:

```
123456
```

After successful pairing the device shows up in your phone's paired
devices list, and apps like Sideband can connect without further prompts.

### Changing the PIN

The PIN is stored in the device's EEPROM. To change it, connect via USB
and use the webflasher's **"Bluetooth Pairing PIN"** panel: read the
current PIN, type a new 6-digit PIN, and click **Write PIN**. You'll need
to reboot the device for the change to take effect, and re-pair from
your phone (remove the old pairing first in Android's Bluetooth settings).

It can also be changed programmatically via the KISS command `CMD_BLE_PIN`
(0x70): send a 1-byte payload `0x00` to read the current PIN, or 6 ASCII
digits to store a new one.

**Security note:** The default PIN is weak and identical across every
flashed device. If you're using BLE in an environment where pairing
hijack is a concern, change it to something unique before pairing.

## Provisioning with rnodeconf

The firmware supports EEPROM emulation for `rnodeconf` provisioning. The virtual EEPROM is backed by LittleFS on the nRF52840's internal flash.

```bash
rnodeconf /dev/ttyACM0 --autoinstall
```

## Webflasher

A browser-based flasher and provisioning tool is included in `docs/`. It uses the Web Serial API (Chrome/Edge/Opera) to:

- Flash firmware via nRF52 DFU (1200-baud touch reset)
- Provision device identity (product, model, serial, checksum) to EEPROM
- Configure radio parameters (frequency, bandwidth, SF, CR, TX power)
- Read device info, blink LED, dump EEPROM

To use it locally, serve the `docs/` directory with any static HTTP server. No build step required.

```bash
cd docs && python -m http.server 8080
# Then open http://localhost:8080
```

It's also deployed via GitHub Pages at the repo's Pages URL (Settings > Pages > Source: `/docs` from `master`).

## Features

- Full RNode KISS protocol (CMD_DETECT, CMD_FW_VERSION, CMD_PLATFORM, radio config, CMD_DATA)
- Split-packet support for payloads > 254 bytes (RNode-compatible two-frame reassembly)
- Non-blocking radio (async TX + poll-driven state machine) so LoRa airtime never starves the BLE link
- CSMA/CA with carrier-sense before transmit
- Airtime accounting with host-configurable short/long caps (CMD_ST_ALOCK / CMD_LT_ALOCK) — TX is held when over budget
- Channel-time, physical-layer, and CSMA telemetry (CMD_STAT_CHTM / CMD_STAT_PHYPRM / CMD_STAT_CSMA)
- Radio config lock (CMD_RADIO_LOCK) and implicit-header toggle (CMD_IMPLICIT)
- Radio config persistence (CMD_CONF_SAVE/DELETE)
- Battery voltage reporting (CMD_STAT_BAT)
- BLE transport with configurable pairing PIN (default `123456`, change via webflasher or CMD_BLE_PIN)
- EEPROM emulation on LittleFS (ROM_READ/ROM_WRITE for rnodeconf)
- RSSI/SNR reporting on every received packet
- TX flow control (CMD_READY)
- SPI recovery with automatic radio reset on failure
- UF2 output for drag-and-drop flashing on supported boards

## Architecture

All board differentiation is via macros in `include/board/<name>.h`. No `#ifdef BOARD_MODEL` chains in source files. Adding a new SX1262-based board requires only:

1. A new board header in `include/board/`
2. A new `[env:]` section in `platformio.ini`

See [CLAUDE.md](CLAUDE.md) for detailed architecture documentation.

## Related Projects

- [reticulum-lora-repeater](https://github.com/thatSFguy/reticulum-lora-repeater) — Same hardware, different personality: runs Reticulum on-device as an autonomous transport node.

## License

See [LICENSE](LICENSE).
