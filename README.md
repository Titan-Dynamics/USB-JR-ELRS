# USB JR Bay for ELRS

Intended to simulate a JR Bay by forwarding joystick commands from a desktop app to an ESP32 which feeds CRSF to a TX Module. 3D-Printable JR bay STL files are included.

- Input: 16 channels from host over USB CDC
- Output: CRSF RC channels to JR bay at 1.87M baud
- Channels: 16 (11-bit packed, CRSF range 172..1811)

## Hardware

- Target MCU: Seeed Studio Xiao-ESP32-S3 or similar ESP32-S3 board
- TX Module: ExpressLRS JR-bay module
- Power: TX module powered separately (e.g., 8.4V). ESP32 via USB.
- Ground: Shared GND between ESP32 and TX module is REQUIRED.
- Signal: One UART TX line (e.g. D4 on Xiao) from ESP32 to module’s JR signal pin (module RX).

Default pin mapping and baud (change in `crsf_handler.h`):
- `#define CRSF_UART_PIN   5`
- `#define CRSF_BAUD       1870000`

Wiring summary:
- ESP32 `GND` → Module `GND`
- ESP32 `D4` (or configured pin) → Module JR signal input (CRSF)
- Module VBAT (e.g., 2S ~8.4V) → Module power input

Note: Make sure the chosen ESP32 pin is 3.3V logic and wired to the module’s signal input. Most JR modules accept CRSF on their single signal pin (the same pin used for PPM on legacy radios).

## Build (PlatformIO)

1. Install PlatformIO (VS Code extension or CLI).
2. Open this folder in VS Code, or `cd` into it for CLI.
3. Build and upload:
   - VS Code: "PlatformIO: Upload"
   - CLI: `pio run -t upload`

`platformio.ini` is configured for `seeed_xiao_esp32s3`. Adjust if your board differs.

## Desktop Feeder (Joystick -> USB)

Simple Python feeder that reads a joystick and streams 16 channels to the board.

- Location: `tools/feeder/`
- Requirements: Python 3.9+, `pip install -r tools/feeder/requirements.txt`
- Auto-detects joystick if `--joy-index` is not provided and prints detected devices.
- Auto-connects to the selected COM port (or previously selected).
- Allows configuring channels as axes or buttons with toggle/rotary options.
- Prints link stats if TX module reports them over serial.

<img width="1402" height="932" alt="image" src="https://github.com/user-attachments/assets/b8a4ee37-5fc0-4a0b-8187-bf593146136b" />

## Host Protocols

Two simple input formats are supported over USB CDC (`Serial`):

1) ASCII line: 16 integers in microseconds 1000..2000 separated by comma or space, newline-terminated.
   - Example: `1500,1500,1500,1500,1000,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500\n`

2) Binary frame: 
   - Header: `0xA5 0x5A`
   - Payload: 16 × little-endian `uint16_t` values (1000..2000)
   - Checksum: 1 byte, sum of all payload bytes mod 256

On valid frame, the firmware updates all 16 channels and starts streaming CRSF to the module. If no frame arrives for >1000 ms, no CRSF is outputted.

## Safety

- Always verify wiring with power disconnected.
- Ensure shared ground between ESP32 and module.
- The module expects 3.3V logic. Do not feed 5V logic.
- Power the module per its specs (e.g., 2S). ESP32 is powered by USB.
- Designed to honor failsafe scenarios such as disconnected joystick or ESP32 USB cable - which leads to RX receiving no CRSF and triggering failsafe on most flight control software such as Ardupilot or Betaflight. Reconnection of joystick and ESP32 usb cable is automatic requiring no manual intevention or "connect button" type clicks from the user.
- Use at your own risk and test all failsafe scenarios prior to flight.

## 3D-Printed JR Bay
- Built to be used with the Seeed Studio Xiao ESP32-S3 and DROK LM2596 Buck Converter to provide 8.4V. A mountable XT60 can also be purchased, or a regular XT60 can be glued to the port opening.
<img width="1600" height="1256" alt="image" src="https://github.com/user-attachments/assets/69a5f511-d908-49d9-918d-66ca5ab8744b" />
<img width="1600" height="1200" alt="image" src="https://github.com/user-attachments/assets/bf28fecf-215a-4c77-9279-5b32b4d4da69" />
