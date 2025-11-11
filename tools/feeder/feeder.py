#!/usr/bin/env python3
# ELRS Calibration GUI + Telemetry (PyQt5)
# pip install pyqt5 pygame pyserial

import sys, json, time, struct, threading
from PyQt5 import QtWidgets, QtCore
import pygame
import serial

HEADER0, HEADER1 = 0x55, 0xAA
TYPE_CHANNELS = 0x01
TYPE_TEL      = 0x02
TYPE_DEBUG    = 0x03
TYPE_TEL_RAW  = 0x04

CHANNELS = 16
SEND_HZ = 60

DEFAULT_PORT = "COM11" if sys.platform.startswith("win") else "/dev/ttyACM0"
DEFAULT_BAUD = 115200

def crc8_d5(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def build_frame(ftype: int, payload: bytes) -> bytes:
    body = bytes([ftype]) + payload
    length = len(body)
    hdr = bytes([HEADER0, HEADER1, length & 0xFF, (length >> 8) & 0xFF])
    crc = crc8_d5(body)
    return hdr + body + bytes([crc])

# ---- Config ----
DEFAULT_CFG = {
    "serial_port": DEFAULT_PORT,
    "serial_baud": DEFAULT_BAUD,
    "channels": [
        {"src": "axis", "idx": 0, "inv": False, "min": 200, "center": 1024, "max": 1847},  # A
        {"src": "axis", "idx": 1, "inv": False, "min": 200, "center": 1024, "max": 1847},  # E
        {"src": "axis", "idx": 2, "inv": False, "min": 200, "center": 1024, "max": 1847},  # T
        {"src": "axis", "idx": 3, "inv": False, "min": 200, "center": 1024, "max": 1847},  # R
    ] + [{"src": "const", "idx": 0, "inv": False, "min": 0, "center": 0, "max": 2047} for _ in range(12)]
}

SRC_CHOICES = ["axis", "button", "const"]

# -------------------------------------------------------------------

class SerialThread(QtCore.QObject):
    telemetry = QtCore.pyqtSignal(dict)
    debug = QtCore.pyqtSignal(str)
    raw_tel = QtCore.pyqtSignal(int)

    def __init__(self, port, baud):
        super().__init__()
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.running = True
        self.raw_tel_count = 0

    def close(self):
        self.running = False
        try:
            self.ser.close()
        except:
            pass

    def send_channels(self, ch16):
        payload = bytearray()
        for v in ch16:
            v = max(0, min(2047, v))
            payload += struct.pack("<H", v)
        pkt = build_frame(TYPE_CHANNELS, bytes(payload))
        try:
            self.ser.write(pkt)
        except Exception as e:
            self.debug.emit(f"Serial write error: {e}")

    def run(self):
        buf = bytearray()
        while self.running:
            try:
                data = self.ser.read(256)
                if data:
                    buf.extend(data)
                    while True:
                        p = buf.find(bytes([HEADER0, HEADER1]))
                        if p < 0 or len(buf) < p + 5:
                            if p > 0:
                                del buf[:p]
                            break
                        if p > 0:
                            del buf[:p]
                        if len(buf) < 5:
                            break
                        length = buf[2] | (buf[3] << 8)
                        total = 4 + length + 1
                        if len(buf) < total:
                            break
                        ftype = buf[4]
                        payload = bytes(buf[5 : 4 + length])
                        crc = buf[4 + length]
                        if crc8_d5(bytes([ftype]) + payload) == crc:
                            self._handle_frame(ftype, payload)
                        del buf[:total]
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.debug.emit(f"Serial read error: {e}")
                time.sleep(0.2)

    def _handle_frame(self, t, payload):
        if t == TYPE_TEL and len(payload) >= 10:
            r = {
                "1RSS": int(struct.unpack("b", payload[0:1])[0]),
                "2RSS": int(struct.unpack("b", payload[1:2])[0]),
                "LQ": payload[2],
                "RSNR": int(struct.unpack("b", payload[3:4])[0]),
                "RFMD": payload[4],
                "TPWR": payload[5],
                "TRSS": int(struct.unpack("b", payload[6:7])[0]),
                "TLQ": payload[7],
                "TSNR": int(struct.unpack("b", payload[8:9])[0]),
                "FLAGS": payload[9],
            }
            self.telemetry.emit(r)
        elif t == TYPE_DEBUG:
            try:
                self.debug.emit(payload.decode(errors="ignore"))
            except:
                pass
        elif t == TYPE_TEL_RAW:
            self.raw_tel_count += 1
            self.raw_tel.emit(self.raw_tel_count)

# -------------------------------------------------------------------
class Joy(QtCore.QObject):
    status = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        pygame.init()
        pygame.joystick.init()
        self.j = None
        self.name = "None"

        # periodic scanner
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._scan)
        self.timer.start(2000)

        self.status.emit("Scanning for joystick...")

    def _scan(self):
        # Only scan if no joystick is currently active
        if self.j is not None:
            # Check that it still exists
            count = pygame.joystick.get_count()
            try:
                # Safely access name to confirm it hasn't gone invalid
                _ = self.j.get_name()
            except pygame.error:
                # joystick object became invalid
                self.status.emit(f"Joystick '{self.name}' disconnected. Scanning...")
                self.j = None
                self.name = "None"
                pygame.joystick.quit()
                pygame.joystick.init()
            return

        # If we get here, there is no active joystick — scan for one
        pygame.joystick.quit()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count == 0:
            self.status.emit("Scanning for joystick...")
        else:
            try:
                self.j = pygame.joystick.Joystick(0)
                self.j.init()
                self.name = self.j.get_name()
                self.status.emit(f"Joystick connected: {self.name}")
            except Exception as e:
                self.status.emit(f"Joystick init error: {e}")
                self.j = None
                self.name = "None"

    def read(self):
        pygame.event.pump()
        axes, btns = [], []
        if self.j:
            try:
                for i in range(self.j.get_numaxes()):
                    axes.append(self.j.get_axis(i))
                for i in range(self.j.get_numbuttons()):
                    btns.append(1 if self.j.get_button(i) else 0)
            except pygame.error:
                # Lost joystick during read
                self.status.emit(f"Joystick '{self.name}' lost. Scanning...")
                self.j = None
                self.name = "None"
        return axes, btns

# -------------------------------------------------------------------

def map_axis_to_0_2047(val, inv, mn, ct, mx):
    if inv:
        val = -val
    if val >= 0:
        return int(ct + val * (mx - ct))
    else:
        return int(ct + val * (ct - mn))

class ChannelRow(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal()
    mapRequested = QtCore.pyqtSignal(object)

    def __init__(self, idx, cfg):
        super().__init__()
        self.idx = idx
        layout = QtWidgets.QGridLayout(self)
        name = f"CH{idx+1}"
        if idx < 4:
            name += f" ({'AETR'[idx]})"

        self.lbl = QtWidgets.QLabel(name)
        self.bar = QtWidgets.QProgressBar(); self.bar.setRange(0,2047)
        self.val = QtWidgets.QLabel("1024")

        self.src = QtWidgets.QComboBox(); self.src.addItems(SRC_CHOICES)
        self.src.setCurrentText(cfg.get("src","const"))

        self.idxBox = QtWidgets.QSpinBox(); self.idxBox.setRange(0,63); self.idxBox.setValue(cfg.get("idx",0))
        self.inv = QtWidgets.QCheckBox("inv"); self.inv.setChecked(cfg.get("inv",False))
        self.toggleBox = QtWidgets.QCheckBox("Toggle"); self.toggleBox.setChecked(cfg.get("toggle", False))

        self.minBox = QtWidgets.QSpinBox(); self.minBox.setRange(0,2047); self.minBox.setValue(cfg.get("min",200))
        self.midBox = QtWidgets.QSpinBox(); self.midBox.setRange(0,2047); self.midBox.setValue(cfg.get("center",1024))
        self.maxBox = QtWidgets.QSpinBox(); self.maxBox.setRange(0,2047); self.maxBox.setValue(cfg.get("max",1847))

        self.mapBtn = QtWidgets.QPushButton("Map…")

        layout.addWidget(self.lbl, 0,0)
        layout.addWidget(self.bar, 0,1,1,6)
        layout.addWidget(self.val, 0,7)

        layout.addWidget(QtWidgets.QLabel("src"), 1,0)
        layout.addWidget(self.src, 1,1)
        layout.addWidget(QtWidgets.QLabel("idx"), 1,2)
        layout.addWidget(self.idxBox, 1,3)
        layout.addWidget(self.inv, 1,4)
        layout.addWidget(QtWidgets.QLabel("min"), 1,5)
        layout.addWidget(self.minBox, 1,6)
        layout.addWidget(QtWidgets.QLabel("mid"), 1,7)
        layout.addWidget(self.midBox, 1,8)
        layout.addWidget(QtWidgets.QLabel("max"), 1,9)
        layout.addWidget(self.maxBox, 1,10)
        layout.addWidget(self.mapBtn, 1,11)
        layout.addWidget(self.toggleBox, 1,12)

        for w in [self.src,self.idxBox,self.inv,self.minBox,self.midBox,self.maxBox,self.toggleBox]:
            if isinstance(w, QtWidgets.QAbstractButton):
                w.toggled.connect(self.changed.emit)
            else:
                w.currentIndexChanged.connect(self.changed.emit) if isinstance(w, QtWidgets.QComboBox) else w.valueChanged.connect(self.changed.emit)

        self.mapBtn.clicked.connect(self._on_map)

    def _on_map(self):
        self.mapRequested.emit(self)

    def compute(self, axes, btns):
        src = self.src.currentText()
        idx = self.idxBox.value()
        inv = self.inv.isChecked()
        mn  = self.minBox.value()
        ct  = self.midBox.value()
        mx  = self.maxBox.value()
        # Edge-detect/toggle state for button source
        if not hasattr(self, "_btn_last"):
            self._btn_last = 0
        if not hasattr(self, "_btn_toggle_state"):
            self._btn_toggle_state = 0
        if not hasattr(self, "_prev_btn_idx"):
            self._prev_btn_idx = idx
        if src == "axis":
            v = axes[idx] if idx < len(axes) else 0.0
            out = map_axis_to_0_2047(v, inv, mn, ct, mx)
        elif src == "button":
            if idx != self._prev_btn_idx:
                self._btn_last = btns[idx] if idx < len(btns) else 0
                self._prev_btn_idx = idx
            v = btns[idx] if idx < len(btns) else 0
            if self.toggleBox.isChecked():
                if self._btn_last == 0 and v == 1:
                    self._btn_toggle_state = 0 if self._btn_toggle_state else 1
                eff = self._btn_toggle_state
            else:
                eff = v
            out = mx if (eff ^ inv) else mn
            self._btn_last = v
        else:
            out = ct
        self.bar.setValue(out)
        self.val.setText(str(out))
        return out

    def to_cfg(self):
        return {
            "src": self.src.currentText(),
            "idx": self.idxBox.value(),
            "inv": self.inv.isChecked(),
            "toggle": self.toggleBox.isChecked(),
            "min": self.minBox.value(),
            "center": self.midBox.value(),
            "max": self.maxBox.value(),
        }

    def set_mapping(self, src: str, idx: int):
        if src in SRC_CHOICES:
            self.src.setCurrentText(src)
        self.idxBox.setValue(idx)

# -------------------------------------------------------------------

class Main(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ELRS Calibrator + Link Stats")
        self.resize(1100, 700)
        self.cfg = DEFAULT_CFG.copy()
        self._load_cfg()

        layout = QtWidgets.QVBoxLayout(self)

        # Mapping state (for joystick-to-channel learn)
        self.mapping_row = None
        self.mapping_baseline = ([], [])
        self.mapping_started_at = 0.0

        # Serial thread
        self.serThread = SerialThread(self.cfg["serial_port"], self.cfg["serial_baud"])
        self.thread = threading.Thread(target=self.serThread.run, daemon=True)
        self.thread.start()
        self.serThread.telemetry.connect(self.onTel)
        self.serThread.debug.connect(self.onDebug)
        self.serThread.raw_tel.connect(self.onRawTel)

        # Joystick (auto-scanning)
        self.joy = Joy()
        self.joy.status.connect(self.onDebug)

        # Channels (scrollable)
        self.rows = []
        grid = QtWidgets.QGridLayout()
        for i in range(CHANNELS):
            row = ChannelRow(i, self.cfg["channels"][i] if i < len(self.cfg["channels"]) else DEFAULT_CFG["channels"][0])
            row.changed.connect(self.save_cfg)
            row.mapRequested.connect(self.begin_mapping)
            self.rows.append(row)
            grid.addWidget(row, i, 0)

        ch_container = QtWidgets.QWidget()
        ch_container.setLayout(grid)

        ch_scroll = QtWidgets.QScrollArea()
        ch_scroll.setWidgetResizable(True)
        ch_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        ch_scroll.setWidget(ch_container)
        ch_scroll.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        layout.addWidget(ch_scroll)

        # Telemetry
        tel = QtWidgets.QHBoxLayout()
        self.telLabels = {}
        for key in ["1RSS","2RSS","RSNR","TRSS","TSNR","LQ","TLQ","RFMD","TPWR"]:
            box = QtWidgets.QGroupBox(key)
            lab = QtWidgets.QLabel("--")
            lab.setAlignment(QtCore.Qt.AlignCenter)
            f = lab.font(); f.setPointSize(14); lab.setFont(f)
            v = QtWidgets.QVBoxLayout(box); v.addWidget(lab)
            tel.addWidget(box)
            self.telLabels[key] = lab
        self.rawCount = QtWidgets.QLabel("RAW: 0")
        tel.addWidget(self.rawCount)
        layout.addLayout(tel)

        # Log (fixed height)
        self.log = QtWidgets.QPlainTextEdit(); self.log.setReadOnly(True)
        self.log.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.log.setFixedHeight(140)
        layout.addWidget(self.log)

        # Timer loop
        # Only the channels area should expand/contract on resize
        # ch_scroll is the first item added to the main layout
        layout.setStretch(0, 1)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(int(1000 / SEND_HZ))

    def onTel(self, d):
        for k, v in d.items():
            if k in self.telLabels:
                self.telLabels[k].setText(str(v))

    def onRawTel(self, n):
        self.rawCount.setText(f"RAW: {n}")

    def onDebug(self, s):
        self.log.appendPlainText(s)

    def tick(self):
        axes, btns = self.joy.read()

        # Mapping mode: detect next button press or large axis move
        if self.mapping_row is not None:
            base_axes, base_btns = self.mapping_baseline
            detected = None
            # Button press has priority
            if btns and base_btns:
                for i in range(min(len(btns), len(base_btns))):
                    if base_btns[i] == 0 and btns[i] == 1:
                        detected = ("button", i)
                        break
            # Axis movement if no button detected
            if detected is None and axes and base_axes:
                best_i, best_d = -1, 0.0
                for i in range(min(len(axes), len(base_axes))):
                    d = abs(axes[i] - base_axes[i])
                    if d > best_d:
                        best_d, best_i = d, i
                if best_i >= 0 and best_d > 0.35:
                    detected = ("axis", best_i)

            if detected is not None:
                src, idx = detected
                self.mapping_row.set_mapping(src, idx)
                try:
                    self.mapping_row.mapBtn.setText("Map…")
                    self.mapping_row.mapBtn.setEnabled(True)
                except Exception:
                    pass
                self.onDebug(f"Mapped CH{self.mapping_row.idx+1} to {src}[{idx}]")
                self.mapping_row = None
                self.save_cfg()
            elif time.time() - self.mapping_started_at > 8.0:
                # Timeout
                try:
                    self.mapping_row.mapBtn.setText("Map…")
                    self.mapping_row.mapBtn.setEnabled(True)
                except Exception:
                    pass
                self.onDebug("Mapping timed out; try again.")
                self.mapping_row = None

        ch = [r.compute(axes, btns) for r in self.rows]
        self.serThread.send_channels(ch)

    def save_cfg(self):
        self.cfg["channels"] = [r.to_cfg() for r in self.rows]
        self._save_cfg_disk()
        self.onDebug("Config saved")

    def _save_cfg_disk(self):
        try:
            with open("calib.json", "w") as f:
                json.dump(self.cfg, f, indent=2)
        except Exception as e:
            self.onDebug(f"Save error: {e}")

    def _load_cfg(self):
        try:
            with open("calib.json", "r") as f:
                disk = json.load(f)
            self.cfg.update(disk)
            chs = self.cfg.get("channels", [])
            if len(chs) < CHANNELS:
                chs += [DEFAULT_CFG["channels"][0]] * (CHANNELS - len(chs))
            self.cfg["channels"] = chs[:CHANNELS]
        except:
            pass

    def closeEvent(self, e):
        try:
            self.serThread.close()
        except:
            pass
        e.accept()

    # --- Mapping helpers ---
    def begin_mapping(self, row: ChannelRow):
        # If another mapping is active, cancel it visually
        if self.mapping_row is not None and hasattr(self.mapping_row, "mapBtn"):
            try:
                self.mapping_row.mapBtn.setText("Map…")
                self.mapping_row.mapBtn.setEnabled(True)
            except Exception:
                pass
        self.mapping_row = row
        self.mapping_baseline = self.joy.read()
        self.mapping_started_at = time.time()
        try:
            row.mapBtn.setText("Listening…")
            row.mapBtn.setEnabled(False)
        except Exception:
            pass
        self.onDebug(f"Move an axis or press a button to map CH{row.idx+1} …")

# -------------------------------------------------------------------

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = Main()
    w.show()
    sys.exit(app.exec_())
