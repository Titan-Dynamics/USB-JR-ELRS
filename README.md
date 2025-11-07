# ELRS JR Bridge (ESP32-S3 / Arduino Nano ESP32)

ESP32-S3 firmware to receive 16-channel RC data from a host (over USB CDC) and forward it to an ExpressLRS TX module via the JR bay using CRSF. Intended to simulate an EdgeTX radio feeding a JR bay module.

- Input: 16 channels from host over USB CDC
- Output: CRSF RC channels to JR bay at 420000 baud
- Channels: 16 (11-bit packed, CRSF range 172..1811)

## Hardware

- Target MCU: Arduino Nano ESP32 (ESP32-S3) or similar ESP32-S3 board
- TX Module: ExpressLRS JR-bay module
- Power: TX module powered separately (e.g., 8.4V). ESP32 via USB.
- Ground: Shared GND between ESP32 and TX module is REQUIRED.
- Signal: One UART TX line from ESP32 to module’s JR signal pin (module RX).

Default pin mapping (change in `src/main.cpp`):
- `JR_TX_PIN = -1` (uses board default TX1)
- `JR_RX_PIN = -1` (unused)

Wiring summary:
- ESP32 `GND` → Module `GND`
- ESP32 `GPIO17` (or configured pin) → Module JR signal input (CRSF)
- Module VBAT (e.g., 2S ~8.4V) → Module power input

Note: Make sure the chosen ESP32 pin is 3.3V logic and wired to the module’s signal input. Most JR modules accept CRSF on their single signal pin (the same pin used for PPM on legacy radios).

## Build (PlatformIO)

1. Install PlatformIO (VS Code extension or CLI).
2. Open this folder `elrs-jr-bridge` in VS Code, or `cd` into it for CLI.
3. Build and upload:
   - VS Code: "PlatformIO: Upload"
   - CLI: `pio run -t upload`

`platformio.ini` is configured for `arduino_nano_esp32`. Adjust if your board differs.

## Desktop Feeder (Joystick -> USB)

Simple Python feeder that reads a joystick and streams 16 channels to the board.

- Location: `tools/feeder/`
- Requirements: Python 3.9+, `pip install -r tools/feeder/requirements.txt`
- Auto-detects joystick if `--joy-index` is not provided and prints detected devices.
- Prints periodic status lines showing axes and CH1..CH6 values.

Example (Windows):

```
python tools/feeder/feeder.py --mode binary --rate 150
```

Example (Linux):

```
python3 tools/feeder/feeder.py --mode binary --rate 150
```

Useful options:
- `--joy-index` pick a specific joystick (auto-detects if omitted)
- `--axis-roll 0 --axis-pitch 1 --axis-throttle 2 --axis-yaw 3` map axes
- `--invert-pitch` etc. to flip directions
- `--arm-button 0` sets CH5 high when pressed
- `--mode-hat 0` uses hat left/center/right for CH6 (1000/1500/2000)
- `--mode ascii` to send ASCII lines instead of binary
- `--list-ports` to enumerate serial ports and exit
  - If `--port` is omitted, the feeder auto-detects the best matching port.

### Windows notes

- If you see write errors like "Write timeout" or "The device does not recognize the command.", the chosen COM port is likely not the ESP32 CDC port (e.g., a Bluetooth COM port).
- Use `--list-ports` to inspect ports, or specify `--port COMx` explicitly.
- Unplug/replug the board and watch which COM port appears.
- The feeder probes ports for the firmware banner; if found, it prefers that port automatically.

Default mapping:
- CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw
- CH5 arm (button), CH6 mode (hat), others at 1500

## Host Protocols

Two simple input formats are supported over USB CDC (`Serial`):

1) ASCII line: 16 integers in microseconds 1000..2000 separated by comma or space, newline-terminated.
   - Example: `1500,1500,1500,1500,1000,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500\n`

2) Binary frame: 
   - Header: `0xA5 0x5A`
   - Payload: 16 × little-endian `uint16_t` values (1000..2000)
   - Checksum: 1 byte, sum of all payload bytes mod 256

On valid frame, the firmware updates all 16 channels and starts streaming CRSF to the module at ~150 Hz. If no frame arrives for 500 ms, channels fail back to safe defaults (1500, with CH5 low at 1000).

## CRSF Details

- Baud: 420000 8N1
- Frame type: RC Channels Packed (0x16)
- Address: 0xC8 (module)
- Length field: 24 (includes type + 22-byte payload + 1-byte CRC)
- CRC: CRC8 poly 0xD5 over `type + payload`
- Channel map: 16 channels, CRSF 11-bit values in [172..1811]
  - Mapping from microseconds (1000..2000) to CRSF: linear to [172..1811]

## Tuning

- Change `JR_TX_PIN` and `JR_RX_PIN` in `src/main.cpp` if needed.
- Adjust `CRSF_SEND_RATE_HZ` to set the CRSF output rate.
- Adjust `CHANNEL_TIMEOUT_MS` for hold/failsafe window.

## Safety

- Always verify wiring with power disconnected.
- Ensure shared ground between ESP32 and module.
- The module expects 3.3V logic. Do not feed 5V logic.
- Power the module per its specs (e.g., 2S). ESP32 is powered by USB.

## Roadmap

- Optional PPM fallback.
- Optional HID joystick directly on ESP32-S3 (USB device), eliminating the PC-side feeder.
- Telemetry input (CRSF back) if/when needed.
