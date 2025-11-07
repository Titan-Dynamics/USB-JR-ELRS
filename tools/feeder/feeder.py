#!/usr/bin/env python3
import argparse
import sys
import time
from typing import List

try:
    import pygame
    import serial
    from serial.tools import list_ports
except ImportError as e:
    print("Missing dependency:", e)
    print("Install with: pip install -r tools/feeder/requirements.txt")
    sys.exit(1)


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def axis_to_us(val: float, center: bool = True) -> int:
    # val in [-1, 1]
    v = clamp(val, -1.0, 1.0)
    if center:
        # -1..1 -> 1000..2000
        return int(round(1500 + v * 500))
    else:
        # 0..1 -> 1000..2000 (map -1..1 to 0..1 first)
        v01 = (v + 1.0) * 0.5
        return int(round(1000 + v01 * 1000))


def build_ascii(ch: List[int]) -> bytes:
    return (",".join(str(int(clamp(c, 1000, 2000))) for c in ch) + "\n").encode("ascii")


def build_binary(ch: List[int]) -> bytes:
    data = bytearray()
    data += b"\xA5\x5A"
    payload = bytearray()
    for c in ch:
        c = int(clamp(c, 1000, 2000))
        payload += bytes((c & 0xFF, (c >> 8) & 0xFF))
    checksum = sum(payload) & 0xFF
    data += payload
    data.append(checksum)
    return bytes(data)


def main():
    ap = argparse.ArgumentParser(description="Joystick -> ESP32 ELRS JR bridge feeder")
    ap.add_argument("--port", default=None, help="Serial port (e.g., COM5, /dev/ttyACM0). Auto-detect if omitted")
    ap.add_argument("--baud", type=int, default=115200, help="USB CDC baud (default 115200)")
    ap.add_argument("--mode", choices=["ascii", "binary"], default="binary", help="Frame format to device")
    ap.add_argument("--list-ports", action="store_true", help="List available serial ports and exit")
    ap.add_argument("--rate", type=float, default=150.0, help="Update rate Hz (default 150)")
    ap.add_argument("--joy-index", type=int, default=None, help="Joystick index; auto-detect if not provided")

    # Axis mapping
    ap.add_argument("--axis-roll", type=int, default=0, help="Roll axis index (default 0)")
    ap.add_argument("--axis-pitch", type=int, default=1, help="Pitch axis index (default 1)")
    ap.add_argument("--axis-throttle", type=int, default=2, help="Throttle axis index (default 2)")
    ap.add_argument("--axis-yaw", type=int, default=3, help="Yaw axis index (default 3)")
    ap.add_argument("--invert-pitch", action="store_true", help="Invert pitch axis")
    ap.add_argument("--invert-roll", action="store_true", help="Invert roll axis")
    ap.add_argument("--invert-yaw", action="store_true", help="Invert yaw axis")
    ap.add_argument("--center-Throttle", action="store_true", dest="center_throttle", help="Treat throttle as centered axis (default off)")
    ap.add_argument("--center-throttle", action="store_true", dest="center_throttle", help="Alias for --center-Throttle")

    # Status output
    ap.add_argument("--no-status", action="store_true", help="Disable periodic status lines")

    # Buttons / hats
    ap.add_argument("--arm-button", type=int, default=0, help="Button index for CH5 arm (default 0)")
    ap.add_argument("--mode-hat", type=int, default=0, help="Hat index for CH6 modes (default 0)")

    args = ap.parse_args()

    # If only listing ports, do that first and exit without requiring a joystick
    if args.list_ports:
        ports = list(list_ports.comports())
        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device} | desc='{p.description}' mfr='{p.manufacturer}' prod='{p.product}' vid={hex(p.vid) if p.vid else None} pid={hex(p.pid) if p.pid else None}")
        return

    pygame.init()
    pygame.joystick.init()
    js_count = pygame.joystick.get_count()
    if js_count == 0:
        print("No joystick found.")
        sys.exit(1)

    # Enumerate devices
    print("Detected joysticks:")
    for i in range(js_count):
        tmp = pygame.joystick.Joystick(i)
        tmp.init()
        print(f"  [{i}] {tmp.get_name()} | axes={tmp.get_numaxes()} buttons={tmp.get_numbuttons()} hats={tmp.get_numhats()}")

    # Pick joystick: user index or best match by most axes/buttons
    if args.joy_index is None:
        # Choose device with max (axes, buttons, hats)
        best_i = 0
        best_key = (-1, -1, -1)
        for i in range(js_count):
            dev = pygame.joystick.Joystick(i)
            dev.init()
            key = (dev.get_numaxes(), dev.get_numbuttons(), dev.get_numhats())
            if key > best_key:
                best_key = key
                best_i = i
        joy_index = best_i
        print(f"Auto-selected joystick index {joy_index}.")
    else:
        if args.joy_index < 0 or args.joy_index >= js_count:
            print(f"Joystick index {args.joy_index} out of range (have {js_count})")
            sys.exit(1)
        joy_index = args.joy_index

    js = pygame.joystick.Joystick(joy_index)
    js.init()
    print(f"Using joystick: {js.get_name()} ({js.get_numaxes()} axes, {js.get_numbuttons()} buttons, {js.get_numhats()} hats)")

    # Serial port selection
    def probe_is_bridge(dev_path: str, baud: int = 115200) -> bool:
        try:
            s = serial.Serial(dev_path, baudrate=baud, timeout=0.1, write_timeout=0.5)
            # Try a DTR/RTS pulse to trigger banner
            try:
                s.dtr = False
                s.rts = False
                time.sleep(0.05)
                s.dtr = True
            except Exception:
                pass
            time.sleep(0.2)
            s.reset_input_buffer()
            t0 = time.time()
            buf = b""
            while time.time() - t0 < 1.5:
                chunk = s.read(256)
                if chunk:
                    buf += chunk
                    if b"ELRS JR Bridge" in buf:
                        s.close()
                        return True
            s.close()
        except Exception:
            pass
        return False
    def score_port(p) -> int:
        score = 0
        try:
            vid = p.vid
            pid = p.pid
            mfr = (p.manufacturer or "").lower()
            prod = (p.product or "").lower()
            desc = (p.description or "").lower()
        except Exception:
            vid = pid = None
            mfr = prod = desc = ""

        preferred_vids = {0x303A, 0x2341, 0x2A03}
        if vid in preferred_vids:
            score += 5
        hints = ["arduino", "espressif", "esp32", "nano", "usb jtag", "cp210", "silicon labs", "ftdi", "wch", "ch340", "usb serial"]
        for h in hints:
            if h in mfr or h in prod or h in desc:
                score += 1
        # Path pattern bonus
        path = (p.device or "").lower()
        if any(s in path for s in ["ttyacm", "ttyusb", "com"]):
            score += 1
        # Probe for firmware banner (strong signal)
        try:
            if probe_is_bridge(p.device):
                score += 100
        except Exception:
            pass
        return score

    ports = list(list_ports.comports())

    chosen_port = args.port
    if chosen_port is None:
        if not ports:
            print("No serial ports found. Specify --port if one should exist.")
            sys.exit(1)
        # Pick highest score
        scored = sorted(((score_port(p), p) for p in ports), key=lambda x: x[0], reverse=True)
        top_score = scored[0][0]
        candidates = [p for s, p in scored if s == top_score]
        chosen_port = candidates[0].device
        print("Detected ports:")
        for s, p in scored:
            print(f"  {p.device} score={s} | '{p.description}' mfr='{p.manufacturer}' prod='{p.product}'")
        print(f"Auto-selected serial port: {chosen_port}")

    try:
        ser = serial.Serial(chosen_port, args.baud, timeout=0, write_timeout=1)
        # Stabilize the CDC endpoint and clear buffers
        try:
            ser.dtr = True
            ser.rts = False
        except Exception:
            pass
        time.sleep(0.2)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
    except Exception as e:
        print(f"Failed to open serial '{chosen_port}':", e)
        sys.exit(1)

    period = 1.0 / args.rate if args.rate > 0 else 1.0 / 150.0
    last = 0.0
    print(f"Streaming {args.mode} frames to {chosen_port} at {args.rate:.1f} Hz")
    print("Status shows axes [-1..1] and channels [1000..2000] (CH1..CH6).")

    last_status = 0.0
    err_count = 0
    start_time = time.time()
    while True:
        now = time.time()
        if now - last < period:
            time.sleep(0.001)
            # Still process events
            pygame.event.pump()
            continue
        last = now

        pygame.event.pump()

        # Read axes safely
        def get_axis(i: int) -> float:
            try:
                return float(js.get_axis(i))
            except Exception:
                return 0.0

        roll = get_axis(args.axis_roll)
        pitch = get_axis(args.axis_pitch)
        yaw = get_axis(args.axis_yaw)
        thr = get_axis(args.axis_throttle)

        if args.invert_roll:
            roll = -roll
        if args.invert_pitch:
            pitch = -pitch
        if args.invert_yaw:
            yaw = -yaw

        ch = [1500] * 16
        ch[0] = axis_to_us(roll, center=True)           # CH1 roll
        ch[1] = axis_to_us(pitch, center=True)          # CH2 pitch
        ch[2] = axis_to_us(thr, center=args.center_throttle)  # CH3 throttle
        ch[3] = axis_to_us(yaw, center=True)            # CH4 yaw

        # Arm switch on CH5
        arm = 0
        try:
            arm = js.get_button(args.arm_button)
        except Exception:
            arm = 0
        ch[4] = 2000 if arm else 1000

        # Flight mode on CH6 using hat (left=1000, center=1500, right=2000)
        mode_val = 1500
        try:
            hatx, haty = js.get_hat(args.mode_hat)
            if hatx < 0:
                mode_val = 1000
            elif hatx > 0:
                mode_val = 2000
            else:
                mode_val = 1500
        except Exception:
            pass
        ch[5] = mode_val

        # Build frame
        frame = build_binary(ch) if args.mode == "binary" else build_ascii(ch)

        try:
            ser.write(frame)
            err_count = 0
        except Exception as e:
            err_count += 1
            print("Serial write error:", e)
            time.sleep(0.25)
            # Early repeated errors: try re-detect and switch port
            if err_count >= 5 and (now - start_time) < 5.0:
                print("Multiple write errors. Re-scanning serial ports...")
                ports = list(list_ports.comports())
                if not ports:
                    print("No ports found during re-scan.")
                else:
                    scored = sorted(((score_port(p), p) for p in ports), key=lambda x: x[0], reverse=True)
                    new_port = scored[0][1].device
                    if new_port != chosen_port:
                        print(f"Switching port: {chosen_port} -> {new_port}")
                        try:
                            ser.close()
                        except Exception:
                            pass
                        try:
                            ser = serial.Serial(new_port, args.baud, timeout=0, write_timeout=1)
                            try:
                                ser.dtr = True
                                ser.rts = False
                            except Exception:
                                pass
                            time.sleep(0.2)
                            try:
                                ser.reset_input_buffer()
                                ser.reset_output_buffer()
                            except Exception:
                                pass
                            chosen_port = new_port
                            err_count = 0
                        except Exception as e2:
                            print("Failed to switch port:", e2)

        # Periodic status line (2 Hz)
        if not args.no_status and (now - last_status) >= 0.5:
            last_status = now
            axes_info = f"R={roll:+.2f} P={pitch:+.2f} T={thr:+.2f} Y={yaw:+.2f}"
            ch_info = f"{ch[0]:4d},{ch[1]:4d},{ch[2]:4d},{ch[3]:4d},{ch[4]:4d},{ch[5]:4d}"
            try:
                arm_btn = js.get_button(args.arm_button)
            except Exception:
                arm_btn = 0
            print(f"JS[{joy_index}] {axes_info} | CH1..6: {ch_info} | arm={arm_btn} mode={ch[5]} | {args.mode}@{args.rate:.0f}Hz -> {chosen_port}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
