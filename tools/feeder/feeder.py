#!/usr/bin/env python3
# ELRS Calibration GUI + Telemetry (PyQt5)
# pip install pyqt5 pygame pyserial

import sys, json, time, struct, threading
from PyQt5 import QtWidgets, QtCore
import pygame
import serial
import serial.tools.list_ports

HEADER0, HEADER1 = 0x55, 0xAA
TYPE_CHANNELS = 0x01
TYPE_TEL      = 0x02
TYPE_DEBUG    = 0x03
TYPE_TEL_RAW  = 0x04

CHANNELS = 16
SEND_HZ = 60

DEFAULT_PORT = "COM15" if sys.platform.startswith("win") else "/dev/ttyACM0"
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
    "channels": [{"src": "const", "idx": 0, "inv": False, "min": 1000, "center": 1500, "max": 2000} for _ in range(CHANNELS)]
}

SRC_CHOICES = ["axis", "button", "const"]

# -------------------------------------------------------------------

def get_available_ports():
    """Get list of available COM ports"""
    ports = []
    for port, desc, hwid in sorted(serial.tools.list_ports.comports()):
        ports.append(port)
    return ports

def get_standard_bauds():
    """Get standard baud rates"""
    return [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

# -------------------------------------------------------------------

class SerialThread(QtCore.QObject):
    telemetry = QtCore.pyqtSignal(dict)
    debug = QtCore.pyqtSignal(str)
    raw_tel = QtCore.pyqtSignal(int)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        self.raw_tel_count = 0
        self._connect()

    def _connect(self):
        """Attempt to connect to the serial port"""
        try:
            if self.ser:
                try:
                    self.ser.close()
                except:
                    pass
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.debug.emit(f"Connected to {self.port} @ {self.baud} baud")
        except Exception as e:
            self.debug.emit(f"Failed to connect to {self.port}: {e}")
            self.ser = None

    def reconnect(self, port, baud):
        """Reconnect with new port/baud settings"""
        self.port = port
        self.baud = baud
        self._connect()

    def close(self):
        self.running = False
        try:
            if self.ser:
                self.ser.close()
        except:
            pass

    def send_channels(self, ch16):
        if not self.ser:
            return
        payload = bytearray()
        for v in ch16:
            v = max(0, min(2000, v))
            payload += struct.pack("<H", v)
        pkt = build_frame(TYPE_CHANNELS, bytes(payload))
        try:
            self.ser.write(pkt)
        except Exception as e:
            self.debug.emit(f"Serial write error: {e}")

    def run(self):
        buf = bytearray()
        while self.running:
            if not self.ser:
                time.sleep(0.5)
                continue
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
                self.ser = None
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

def map_axis_to_0_2000(val, inv, mn, ct, mx):
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

        self.lbl = QtWidgets.QLabel(name); self.lbl.setMaximumWidth(50)
        self.nameBox = QtWidgets.QLineEdit(); self.nameBox.setPlaceholderText("Name"); self.nameBox.setMaximumWidth(100)
        default_names = ["Ail", "Elev", "Thr", "Rudd", "Arm", "Mode"]
        default_name = default_names[idx] if idx < len(default_names) else ""
        self.nameBox.setText(cfg.get("name", default_name))
        self.bar = QtWidgets.QProgressBar(); self.bar.setRange(0,2000)
        self.val = QtWidgets.QLabel("1500"); self.val.setMaximumWidth(60); self.val.setMinimumWidth(60)

        self.src = QtWidgets.QComboBox(); self.src.addItems(SRC_CHOICES); self.src.setMaximumWidth(80)
        self.src.setCurrentText(cfg.get("src","const"))

        self.idxBox = QtWidgets.QSpinBox(); self.idxBox.setRange(0,63); self.idxBox.setValue(cfg.get("idx",0)); self.idxBox.setMaximumWidth(60)
        self.inv = QtWidgets.QCheckBox("inv"); self.inv.setChecked(cfg.get("inv",False)); self.inv.setMaximumWidth(60)

        self.toggleBox = QtWidgets.QCheckBox("Toggle"); self.toggleBox.setChecked(cfg.get("toggle", False)); self.toggleBox.setMaximumWidth(80)
        self.rotaryBox = QtWidgets.QCheckBox("Rotary"); self.rotaryBox.setChecked(cfg.get("rotary", False)); self.rotaryBox.setMaximumWidth(80)
        self.rotaryStopsBox = QtWidgets.QSpinBox(); self.rotaryStopsBox.setRange(3,6); self.rotaryStopsBox.setValue(cfg.get("rotary_stops", 3)); self.rotaryStopsBox.setMaximumWidth(60)
        self.rotaryStopsBox.setEnabled(cfg.get("rotary", False))

        self.minBox = QtWidgets.QSpinBox(); self.minBox.setRange(0,2000); self.minBox.setValue(cfg.get("min",1000)); self.minBox.setAlignment(QtCore.Qt.AlignLeft); self.minBox.setMaximumWidth(70)
        self.midBox = QtWidgets.QSpinBox(); self.midBox.setRange(0,2000); self.midBox.setValue(cfg.get("center",1500)); self.midBox.setAlignment(QtCore.Qt.AlignLeft); self.midBox.setMaximumWidth(70)
        self.maxBox = QtWidgets.QSpinBox(); self.maxBox.setRange(0,2000); self.maxBox.setValue(cfg.get("max",2000)); self.maxBox.setAlignment(QtCore.Qt.AlignLeft); self.maxBox.setMaximumWidth(70)

        self.mapBtn = QtWidgets.QPushButton("Map"); self.mapBtn.setMaximumWidth(70)

        # Update progress bar range based on min/max values
        def update_bar_range():
            self.bar.setRange(self.minBox.value(), self.maxBox.value())

        self.minBox.valueChanged.connect(update_bar_range)
        self.maxBox.valueChanged.connect(update_bar_range)
        update_bar_range()  # Set initial range

        # Top row: full-width with scaling elements
        topLayout = QtWidgets.QHBoxLayout()
        topLayout.addWidget(self.lbl)
        topLayout.addWidget(self.nameBox)
        topLayout.addWidget(self.bar, 1)  # progress bar gets stretch
        topLayout.addWidget(self.val)
        layout.addLayout(topLayout, 0, 0, 1, 15)

        # Bottom row: fixed-width controls
        srcLbl = QtWidgets.QLabel("src"); srcLbl.setMaximumWidth(30)
        layout.addWidget(srcLbl, 1,0)
        layout.addWidget(self.src, 1,1)
        idxLbl = QtWidgets.QLabel("idx"); idxLbl.setMaximumWidth(30)
        layout.addWidget(idxLbl, 1,2)
        layout.addWidget(self.idxBox, 1,3)
        layout.addWidget(self.inv, 1,4)
        minLbl = QtWidgets.QLabel("min"); minLbl.setMaximumWidth(30)
        layout.addWidget(minLbl, 1,5)
        layout.addWidget(self.minBox, 1,6)
        midLbl = QtWidgets.QLabel("mid"); midLbl.setMaximumWidth(30)
        layout.addWidget(midLbl, 1,7)
        layout.addWidget(self.midBox, 1,8)
        maxLbl = QtWidgets.QLabel("max"); maxLbl.setMaximumWidth(30)
        layout.addWidget(maxLbl, 1,9)
        layout.addWidget(self.maxBox, 1,10)
        layout.addWidget(self.mapBtn, 1,11)
        layout.addWidget(self.toggleBox, 1,12)
        layout.addWidget(self.rotaryBox, 1,13)
        layout.addWidget(self.rotaryStopsBox, 1,14)

        self.nameBox.textChanged.connect(self.changed.emit)

        for w in [self.src,self.idxBox,self.inv,self.minBox,self.midBox,self.maxBox,self.toggleBox,self.rotaryBox,self.rotaryStopsBox]:
            if isinstance(w, QtWidgets.QAbstractButton):
                w.toggled.connect(self._on_mode_changed)
            else:
                w.currentIndexChanged.connect(self.changed.emit) if isinstance(w, QtWidgets.QComboBox) else w.valueChanged.connect(self.changed.emit)

        self.mapBtn.clicked.connect(self._on_map)

    def _on_map(self):
        self.mapRequested.emit(self)

    def _on_mode_changed(self):
        """Handle toggle/rotary mode changes - ensure only one is selected"""
        if self.toggleBox.isChecked() and self.rotaryBox.isChecked():
            # Only one can be checked - keep the one that was just checked
            # Since we can't easily determine which was just clicked, uncheck rotary if toggle is checked
            self.rotaryBox.blockSignals(True)
            self.rotaryBox.setChecked(False)
            self.rotaryBox.blockSignals(False)

        # Enable/disable rotary stops based on rotary checkbox
        self.rotaryStopsBox.setEnabled(self.rotaryBox.isChecked())
        self.changed.emit()

    def compute(self, axes, btns):
        src = self.src.currentText()
        idx = self.idxBox.value()
        inv = self.inv.isChecked()
        mn  = self.minBox.value()
        ct  = self.midBox.value()
        mx  = self.maxBox.value()
        rotary = self.rotaryBox.isChecked()
        rotary_stops = self.rotaryStopsBox.value()

        # Edge-detect/toggle state for button source
        if not hasattr(self, "_btn_last"):
            self._btn_last = 0
        if not hasattr(self, "_btn_toggle_state"):
            self._btn_toggle_state = 0
        if not hasattr(self, "_btn_rotary_state"):
            self._btn_rotary_state = 0
        if not hasattr(self, "_prev_btn_idx"):
            self._prev_btn_idx = idx

        if src == "axis":
            v = axes[idx] if idx < len(axes) else 0.0
            out = map_axis_to_0_2000(v, inv, mn, ct, mx)
        elif src == "button":
            if idx != self._prev_btn_idx:
                self._btn_last = btns[idx] if idx < len(btns) else 0
                self._prev_btn_idx = idx
            v = btns[idx] if idx < len(btns) else 0

            if rotary:
                # Rotary mode: cycle through stops on button press
                if self._btn_last == 0 and v == 1:
                    self._btn_rotary_state = (self._btn_rotary_state + 1) % rotary_stops
                # Calculate output value based on current stop
                # Divide range into (stops-1) intervals so last stop hits max
                stop_range = mx - mn
                if rotary_stops > 1:
                    stop_value = stop_range / (rotary_stops - 1)
                    out = int(mn + self._btn_rotary_state * stop_value)
                else:
                    out = mn
            elif self.toggleBox.isChecked():
                # Toggle mode: on/off state
                if self._btn_last == 0 and v == 1:
                    self._btn_toggle_state = 0 if self._btn_toggle_state else 1
                eff = self._btn_toggle_state
                out = mx if (eff ^ inv) else mn
            else:
                # Direct mode: button press = max, release = min
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
            "name": self.nameBox.text(),
            "src": self.src.currentText(),
            "idx": self.idxBox.value(),
            "inv": self.inv.isChecked(),
            "toggle": self.toggleBox.isChecked(),
            "rotary": self.rotaryBox.isChecked(),
            "rotary_stops": self.rotaryStopsBox.value(),
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
        self.resize(1400, 900)
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

        # Serial port selection controls
        port_layout = QtWidgets.QHBoxLayout()
        port_layout.addWidget(QtWidgets.QLabel("COM Port:"))

        self.portCombo = QtWidgets.QComboBox()
        self._refresh_port_list()
        self.portCombo.setCurrentText(self.cfg["serial_port"])
        self.portCombo.currentTextChanged.connect(self._on_port_changed)
        port_layout.addWidget(self.portCombo)

        refresh_btn = QtWidgets.QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_port_list)
        refresh_btn.setMaximumWidth(80)
        port_layout.addWidget(refresh_btn)

        port_layout.addWidget(QtWidgets.QLabel("Baud Rate:"))

        self.baudCombo = QtWidgets.QComboBox()
        for baud in get_standard_bauds():
            self.baudCombo.addItem(str(baud), baud)
        self.baudCombo.setCurrentText(str(self.cfg["serial_baud"]))
        self.baudCombo.currentIndexChanged.connect(self._on_baud_changed)
        port_layout.addWidget(self.baudCombo)

        port_layout.addStretch()
        layout.addLayout(port_layout)

        # Joystick (auto-scanning)
        self.joy = Joy()
        self.joy.status.connect(self.onDebug)

        # Channels (scrollable, 2-column layout: 8 on left, 8 on right)
        self.rows = []
        grid = QtWidgets.QGridLayout()
        for i in range(CHANNELS):
            row = ChannelRow(i, self.cfg["channels"][i] if i < len(self.cfg["channels"]) else DEFAULT_CFG["channels"][0])
            row.changed.connect(self.save_cfg)
            row.mapRequested.connect(self.begin_mapping)
            self.rows.append(row)
            # First 8 channels in column 0, next 8 in column 2 (with divider in column 1)
            col = 0 if i < CHANNELS // 2 else 2
            row_idx = i if i < CHANNELS // 2 else i - CHANNELS // 2
            grid.addWidget(row, row_idx, col)

        # Add vertical divider in column 1 (single frame spanning all rows)
        divider = QtWidgets.QFrame()
        divider.setFrameShape(QtWidgets.QFrame.VLine)
        divider.setFrameShadow(QtWidgets.QFrame.Sunken)
        divider.setLineWidth(2)
        grid.addWidget(divider, 0, 1, CHANNELS // 2, 1)

        ch_container = QtWidgets.QWidget()
        ch_container.setLayout(grid)

        ch_scroll = QtWidgets.QScrollArea()
        ch_scroll.setWidgetResizable(True)
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
                    self.mapping_row.mapBtn.setText("Map")
                    self.mapping_row.mapBtn.setEnabled(True)
                except Exception:
                    pass
                self.onDebug(f"Mapped CH{self.mapping_row.idx+1} to {src}[{idx}]")
                self.mapping_row = None
                self.save_cfg()
            elif time.time() - self.mapping_started_at > 8.0:
                # Timeout
                try:
                    self.mapping_row.mapBtn.setText("Map")
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

    def _refresh_port_list(self):
        """Refresh the list of available COM ports"""
        current = self.portCombo.currentText()
        self.portCombo.blockSignals(True)
        self.portCombo.clear()
        ports = get_available_ports()
        for port in ports:
            self.portCombo.addItem(port)
        # If the previous selection still exists, restore it
        if current and self.portCombo.findText(current) >= 0:
            self.portCombo.setCurrentText(current)
        elif ports:
            # Otherwise select the first available port
            self.portCombo.setCurrentIndex(0)
        self.portCombo.blockSignals(False)

    def _on_port_changed(self, port):
        """Handle COM port selection change"""
        if not port:
            return
        self.cfg["serial_port"] = port
        self.serThread.reconnect(port, self.cfg["serial_baud"])
        self.save_cfg()

    def _on_baud_changed(self, index):
        """Handle baud rate selection change"""
        if index < 0:
            return
        baud = self.baudCombo.itemData(index)
        if baud:
            self.cfg["serial_baud"] = baud
            self.serThread.reconnect(self.cfg["serial_port"], baud)
            self.save_cfg()

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
                self.mapping_row.mapBtn.setText("Map")
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
