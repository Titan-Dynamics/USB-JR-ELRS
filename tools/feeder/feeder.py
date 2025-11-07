#!/usr/bin/env python3
# ELRS Calibration GUI + Telemetry (PyQt5)
# pip install pyqt5 pygame pyserial

import sys, json, time, struct, threading
from collections import deque

from PyQt5 import QtWidgets, QtCore, QtGui
import pygame
import serial

HEADER0, HEADER1 = 0x55, 0xAA
TYPE_CHANNELS = 0x01
TYPE_TEL      = 0x02
TYPE_DEBUG    = 0x03
TYPE_TEL_RAW  = 0x04

CHANNELS = 16
SEND_HZ = 60

DEFAULT_PORT = "COM3" if sys.platform.startswith("win") else "/dev/ttyACM0"
DEFAULT_BAUD = 115200

def crc8_d5(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def build_frame(ftype: int, payload: bytes) -> bytes:
    body = bytes([ftype]) + payload
    length = len(body)
    hdr = bytes([HEADER0, HEADER1, length & 0xFF, (length >> 8) & 0xFF])
    crc = crc8_d5(body)
    return hdr + body + bytes([crc])

# ---- Config model ----
DEFAULT_CFG = {
    "serial_port": DEFAULT_PORT,
    "serial_baud": DEFAULT_BAUD,
    "joystick_index": 0,
    # per-channel config: source, axis index, invert, min/center/max (0..2047)
    "channels": [
        {"src":"axis","idx":0,"inv":False,"min":200,"center":1024,"max":1847}, # ch1 A
        {"src":"axis","idx":1,"inv":False,"min":200,"center":1024,"max":1847}, # ch2 E
        {"src":"axis","idx":2,"inv":False,"min":200,"center":1024,"max":1847}, # ch3 T
        {"src":"axis","idx":3,"inv":False,"min":200,"center":1024,"max":1847}, # ch4 R
    ] + [{"src":"const","idx":0,"inv":False,"min":0,"center":0,"max":2047} for _ in range(12)]
}

SRC_CHOICES = ["axis","button","const"]

class SerialThread(QtCore.QObject):
    telemetry = QtCore.pyqtSignal(dict)
    debug     = QtCore.pyqtSignal(str)
    raw_tel   = QtCore.pyqtSignal(int)  # count

    def __init__(self, port, baud):
        super().__init__()
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.running = True
        self.raw_tel_count = 0

    def close(self):
        self.running = False
        try: self.ser.close()
        except: pass

    def send_channels(self, ch16):
        # ch16: list of 16 ints (0..2047)
        payload = bytearray()
        for v in ch16:
            if v<0: v=0
            if v>2047: v=2047
            payload += struct.pack("<H", v)
        pkt = build_frame(TYPE_CHANNELS, bytes(payload))
        try:
            self.ser.write(pkt)
        except Exception as e:
            self.debug.emit(f"Serial write error: {e}")

    def run(self):
        # simple framed reader
        buf = bytearray()
        while self.running:
            try:
                data = self.ser.read(256)
                if data:
                    buf.extend(data)
                    # parse frames
                    while True:
                        # find header
                        p = buf.find(bytes([HEADER0, HEADER1]))
                        if p < 0 or len(buf) < p+5:  # need hdr+len+type at least
                            # trim noise
                            if p > 0: del buf[:p]
                            break
                        if p > 0: del buf[:p]
                        if len(buf) < 5: break
                        length = buf[2] | (buf[3]<<8)
                        total = 4 + length + 1
                        if len(buf) < total: break
                        ftype = buf[4]
                        payload = bytes(buf[5: 4+length])
                        crc = buf[4+length]
                        if crc8_d5(bytes([ftype]) + payload) == crc:
                            self._handle_frame(ftype, payload)
                        del buf[:total]
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.debug.emit(f"Serial read error: {e}")
                time.sleep(0.2)

    def _handle_frame(self, t, payload):
        if t == TYPE_TEL:
            if len(payload) >= 10:
                r = {
                    "1RSS":  int(struct.unpack("b", payload[0:1])[0]),
                    "2RSS":  int(struct.unpack("b", payload[1:2])[0]),
                    "LQ":    payload[2],
                    "RSNR":  int(struct.unpack("b", payload[3:4])[0]),
                    "RFMD":  payload[4],
                    "TPWR":  payload[5],
                    "TRSS":  int(struct.unpack("b", payload[6:7])[0]),
                    "TLQ":   payload[7],
                    "TSNR":  int(struct.unpack("b", payload[8:9])[0]),
                    "FLAGS": payload[9]
                }
                self.telemetry.emit(r)
        elif t == TYPE_DEBUG:
            try:
                self.debug.emit(payload.decode(errors='ignore'))
            except:
                pass
        elif t == TYPE_TEL_RAW:
            self.raw_tel_count += 1
            self.raw_tel.emit(self.raw_tel_count)

class Joy:
    def __init__(self, idx=None):
        pygame.init()
        pygame.joystick.init()

        self.j = None
        self.name = "None"
        count = pygame.joystick.get_count()

        if count == 0:
            print("[WARN] No joysticks detected.")
        else:
            # If user provided an index, use it if valid; otherwise auto-select first
            if idx is not None and idx < count:
                self.j = pygame.joystick.Joystick(idx)
            else:
                self.j = pygame.joystick.Joystick(0)
            self.j.init()
            self.name = self.j.get_name()
            print(f"[INFO] Joystick connected: {self.name}")

    def read(self):
        pygame.event.pump()
        axes, btns = [], []
        if self.j:
            for i in range(self.j.get_numaxes()):
                axes.append(self.j.get_axis(i))
            for i in range(self.j.get_numbuttons()):
                btns.append(1 if self.j.get_button(i) else 0)
        return axes, btns


def map_axis_to_0_2047(val, inv, mn, ct, mx):
    # val in [-1..1] -> [0..2047] using min/center/max
    if inv: val = -val
    # piecewise mapping around center
    if val >= 0:
        hi = mx
        return int(ct + val*(hi-ct))
    else:
        lo = mn
        return int(ct + val*(ct-lo))

class ChannelRow(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal()

    def __init__(self, idx, cfg):
        super().__init__()
        self.idx = idx
        self.cfg = cfg

        layout = QtWidgets.QGridLayout(self)
        name = f"CH{idx+1}"
        if idx==0: name += " (A)"
        if idx==1: name += " (E)"
        if idx==2: name += " (T)"
        if idx==3: name += " (R)"

        self.lbl = QtWidgets.QLabel(name)
        self.bar = QtWidgets.QProgressBar(); self.bar.setRange(0,2047)
        self.val = QtWidgets.QLabel("1024")

        self.src = QtWidgets.QComboBox(); self.src.addItems(SRC_CHOICES)
        self.src.setCurrentText(cfg.get("src","const"))

        self.idxBox = QtWidgets.QSpinBox(); self.idxBox.setRange(0,63); self.idxBox.setValue(cfg.get("idx",0))
        self.inv = QtWidgets.QCheckBox("inv"); self.inv.setChecked(cfg.get("inv",False))

        self.minBox = QtWidgets.QSpinBox(); self.minBox.setRange(0,2047); self.minBox.setValue(cfg.get("min",200))
        self.midBox = QtWidgets.QSpinBox(); self.midBox.setRange(0,2047); self.midBox.setValue(cfg.get("center",1024))
        self.maxBox = QtWidgets.QSpinBox(); self.maxBox.setRange(0,2047); self.maxBox.setValue(cfg.get("max",1847))

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

        for w in [self.src,self.idxBox,self.inv,self.minBox,self.midBox,self.maxBox]:
            if isinstance(w, QtWidgets.QAbstractButton):
                w.toggled.connect(self._emit)
            else:
                w.currentIndexChanged.connect(self._emit) if isinstance(w, QtWidgets.QComboBox) else w.valueChanged.connect(self._emit)

    def _emit(self,*a): self.changed.emit()

    def compute(self, axes, btns):
        src = self.src.currentText()
        idx = self.idxBox.value()
        inv = self.inv.isChecked()
        mn  = self.minBox.value()
        ct  = self.midBox.value()
        mx  = self.maxBox.value()

        if src=="axis":
            v = axes[idx] if idx < len(axes) else 0.0
            out = map_axis_to_0_2047(v, inv, mn, ct, mx)
        elif src=="button":
            v = btns[idx] if idx < len(btns) else 0
            out = mx if (v ^ inv) else mn
        else: # const
            out = ct
        self.bar.setValue(out)
        self.val.setText(str(out))
        return out

    def to_cfg(self):
        return {
            "src": self.src.currentText(),
            "idx": self.idxBox.value(),
            "inv": self.inv.isChecked(),
            "min": self.minBox.value(),
            "center": self.midBox.value(),
            "max": self.maxBox.value()
        }

class Main(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ELRS Calibrator + Link Stats")
        self.resize(1100, 700)

        self.cfg = DEFAULT_CFG.copy()
        self._load_cfg()

        # Serial
        self.serThread = SerialThread(self.cfg["serial_port"], self.cfg["serial_baud"])
        self.thread = threading.Thread(target=self.serThread.run, daemon=True); self.thread.start()
        self.serThread.telemetry.connect(self.onTel)
        self.serThread.debug.connect(self.onDebug)
        self.serThread.raw_tel.connect(self.onRawTel)

        # Joystick
        self.joy = Joy(self.cfg.get("joystick_index",0))

        # UI
        layout = QtWidgets.QVBoxLayout(self)

        # Top bar: port, baud, joystick
        top = QtWidgets.QHBoxLayout()
        self.portEdit = QtWidgets.QLineEdit(self.cfg["serial_port"])
        self.baudEdit = QtWidgets.QLineEdit(str(self.cfg["serial_baud"]))
        self.joyIdx   = QtWidgets.QSpinBox(); self.joyIdx.setRange(0,8); self.joyIdx.setValue(self.cfg.get("joystick_index",0))
        self.btnReconnect = QtWidgets.QPushButton("Reconnect")
        self.btnReconnect.clicked.connect(self.reconnect)
        top.addWidget(QtWidgets.QLabel("Port")); top.addWidget(self.portEdit)
        top.addWidget(QtWidgets.QLabel("Baud")); top.addWidget(self.baudEdit)
        top.addWidget(QtWidgets.QLabel("Joystick")); top.addWidget(self.joyIdx)
        top.addWidget(self.btnReconnect)
        layout.addLayout(top)

        # Channels grid
        self.rows = []
        grid = QtWidgets.QGridLayout()
        for i in range(CHANNELS):
            row = ChannelRow(i, self.cfg["channels"][i] if i < len(self.cfg["channels"]) else DEFAULT_CFG["channels"][0])
            row.changed.connect(self.save_cfg)
            self.rows.append(row)
            grid.addWidget(row, i, 0)
        layout.addLayout(grid)

        # Telemetry panel
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

        # Log / save
        bottom = QtWidgets.QHBoxLayout()
        self.log = QtWidgets.QPlainTextEdit(); self.log.setReadOnly(True)
        layout.addWidget(self.log)
        self.btnSave = QtWidgets.QPushButton("Save Config"); self.btnSave.clicked.connect(self.save_cfg)
        layout.addWidget(self.btnSave)

        # Timer to sample joystick + send channels
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(int(1000/SEND_HZ))

    def reconnect(self):
        try:
            self.serThread.close()
        except: pass
        port = self.portEdit.text().strip()
        baud = int(self.baudEdit.text().strip())
        self.serThread = SerialThread(port, baud)
        self.thread = threading.Thread(target=self.serThread.run, daemon=True); self.thread.start()
        self.serThread.telemetry.connect(self.onTel)
        self.serThread.debug.connect(self.onDebug)
        self.serThread.raw_tel.connect(self.onRawTel)
        self.cfg["serial_port"]=port; self.cfg["serial_baud"]=baud
        self.joy = Joy(self.joyIdx.value())
        self.cfg["joystick_index"]=self.joyIdx.value()
        self._save_cfg_disk()
        self.onDebug(f"Reconnected {port} @ {baud}, joystick {self.cfg['joystick_index']}")

    def onTel(self, d):
        for k,v in d.items():
            if k in self.telLabels:
                self.telLabels[k].setText(str(v))

    def onRawTel(self, n):
        self.rawCount.setText(f"RAW: {n}")

    def onDebug(self, s):
        self.log.appendPlainText(s)

    def tick(self):
        axes, btns = self.joy.read()
        ch = []
        for r in self.rows:
            ch.append(r.compute(axes, btns))
        self.serThread.send_channels(ch)

    def save_cfg(self):
        self.cfg["channels"] = [r.to_cfg() for r in self.rows]
        self.cfg["serial_port"] = self.portEdit.text().strip()
        self.cfg["serial_baud"] = int(self.baudEdit.text().strip())
        self.cfg["joystick_index"] = self.joyIdx.value()
        self._save_cfg_disk()
        self.onDebug("Config saved")

    def _save_cfg_disk(self):
        try:
            with open("calib.json","w") as f: json.dump(self.cfg,f,indent=2)
        except Exception as e:
            self.onDebug(f"Save error: {e}")

    def _load_cfg(self):
        try:
            with open("calib.json","r") as f:
                disk = json.load(f)
            # merge
            self.cfg.update(disk)
            # normalize channels length
            chs = self.cfg.get("channels",[])
            if len(chs) < CHANNELS:
                chs += [DEFAULT_CFG["channels"][0]]*(CHANNELS-len(chs))
            self.cfg["channels"] = chs[:CHANNELS]
        except:
            pass

    def closeEvent(self, e):
        try: self.serThread.close()
        except: pass
        e.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = Main(); w.show()
    sys.exit(app.exec_())
