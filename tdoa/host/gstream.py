"""Serial reader for the gimbal firmware (tdoa/src/gimbal.cpp; keep in sync with its GimbalPacket).

Packet, 125 per second: "SYNC" + bus A + bus B (int16) + "TDOA" + 32 float32 (as tstream.py) + "GMBL" + 18 float32,
all little-endian.
"""
import struct
import threading
from dataclasses import dataclass
import numpy as np
import serial
from tstream import Result, find_port, HOP, BLK16, NF, PKT as TDOA_PKT, FULL_SCALE
from tconfig import MIC_SLOT

GNF = 18
PKT = TDOA_PKT + 4 + 4 * GNF
MODES = ["off", "auto", "jog"]
STATES = ["idle", "tracking", "holding"]


@dataclass
class Gimbal:
    mode: int            # 0 off (no pulses), 1 automatic, 2 jog
    state: int           # 0 idle, 1 tracking, 2 holding (no usable direction: stopped)
    err_pan: float       # pointing error from the mics, deg: + = pan needs to go FORWARD (towards mic 4)
    err_tilt: float      # + = elevation needs to go UP (towards mic 2)
    err: float           # total, deg
    moving: bool         # outside the deadband, driving
    pan_us: float        # pulses sent (0 = none)
    tilt_us: float
    blocked: int         # directions blocked by a hard stop: 1 pan FORWARD, 2 pan REVERSE, 4 elev UP, 8 elev DOWN
    tx_drops: int        # packets the board dropped because nobody was reading
    sim_t: float         # gimbal_sim truth, else 0
    sim_pan: float
    sim_tilt: float
    sim_src_az: float
    sim_src_el: float
    sim_tone: bool
    sim_push: float      # s the sim spent driving an axis into a hard stop
    listening: bool      # servos quiet: mic estimates used (else the tracker is frozen)


def _result(v):
    return Result(v[0:3], v[3], v[4], v[5], v[6], bool(v[7]), v[8], v[9], v[10:16], int(v[16]), v[17], v[18],
                  int(v[19]), v[20:23], v[23:26], v[26], v[27], v[28], v[29], bool(v[30]), int(v[31]))


def _gimbal(v):
    return Gimbal(int(v[0]), int(v[1]), v[2], v[3], v[4], bool(v[5]), v[6], v[7], int(v[8]), int(v[9]),
                  v[10], v[11], v[12], v[13], v[14], bool(v[15]), v[16], bool(v[17]))


class GimbalStream:
    """Background reader. Calls from its thread, per packet: on_packet(r, g) with r = tstream.Result,
    g = Gimbal; on_audio(block) with (4, HOP) float, mic 1..4, full-scale units."""

    def __init__(self, on_packet=None, on_audio=None, port=None):
        self.on_packet, self.on_audio = on_packet, on_audio
        self.ser = serial.Serial()
        self.ser.port = port or find_port()
        self.ser.timeout = 0.1
        self.ser.dtr = False   # DTR/RTS drive the ESP32-S3's reset/boot lines
        self.ser.rts = False
        self.ser.open()
        self.ser.reset_input_buffer()
        self.packets = 0
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, command):
        """E1 auto, E0/S off, JP<us>/JT<us> jog at a pulse offset (repeat every 0.1 s), R restart (sim)."""
        self.ser.write((command + "\n").encode())

    def close(self):
        self.on_packet = self.on_audio = None
        self.ser.close()

    def _run(self):
        rx = b""
        ta = 4 + 2 * BLK16            # offset of "TDOA"
        tg = TDOA_PKT                 # offset of "GMBL"
        while True:
            try:
                rx += self.ser.read(max(1, self.ser.in_waiting))
            except (serial.SerialException, OSError, TypeError):
                return   # port closed
            while True:
                i = rx.find(b"SYNC")
                if i < 0:
                    rx = rx[-3:]
                    break
                if len(rx) < i + PKT:
                    rx = rx[i:]
                    break
                if rx[i + ta:i + ta + 4] != b"TDOA" or rx[i + tg:i + tg + 4] != b"GMBL":
                    rx = rx[i + 1:]
                    continue
                pkt = rx[i:i + PKT]
                rx = rx[i + PKT:]
                self.packets += 1
                cb = self.on_audio
                if cb:
                    w = np.frombuffer(pkt[4:ta], dtype="<i2")
                    a, b = w[:2 * HOP].reshape(-1, 2), w[2 * HOP:].reshape(-1, 2)
                    slots = np.concatenate([a, b], axis=1)
                    cb(slots[:, MIC_SLOT].T / FULL_SCALE)
                cb = self.on_packet
                if cb:
                    v = np.array(struct.unpack(f"<{NF}f", pkt[ta + 4:tg]))
                    gv = np.array(struct.unpack(f"<{GNF}f", pkt[tg + 4:]))
                    if np.all(np.isfinite(v)) and np.all(np.isfinite(gv)):
                        cb(_result(v), _gimbal(gv))
