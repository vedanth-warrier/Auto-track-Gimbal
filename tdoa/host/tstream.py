"""Serial reader for the TDOA firmware (keep in sync with tdoa/src/main.cpp).

Packet, 125 per second: "SYNC" + bus A + bus B (each 256 stereo frames of int16, [left, right])
+ "TDOA" + 32 float32, all little-endian.
Which slot each mic is in comes from MIC_SLOT in src/config.h (0 A left, 1 A right, 2 B left, 3 B right).
"""
import glob
import struct
import threading
from dataclasses import dataclass
import numpy as np
import serial
from tconfig import MIC_SLOT

HOP = 256
BLK16 = HOP * 2 * 2
NF = 32
PKT = 4 + 2 * BLK16 + 4 + 4 * NF
FULL_SCALE = 32768


@dataclass
class Result:
    u: np.ndarray        # unit vector towards the source (array frame); zeros = no tone
    az: float            # deg, anticlockwise from +x (mic 3)
    el: float            # deg above the array plane
    quality: float       # 0..1
    ambiguity: float     # 0..1, best fit anywhere > 30 deg away / best; near 1 = a ghost fits as well
    edge: bool           # best match on the search-cone boundary: source probably outside the cone
    freq: float          # Hz, 0 = no tone
    snr: float           # dB
    tau: np.ndarray      # us, pair delays t_i - t_j for (1,2) (1,3) (1,4) (2,3) (2,4) (3,4)
    resyncs: int
    sim_az: float
    sim_el: float
    track_state: int     # 0 lost, 1 tracking, 2 coasting
    tu: np.ndarray       # tracked direction (unit vector); zeros when lost
    cmd: np.ndarray      # servo command direction (unit vector)
    pan_rate: float      # rad/s commanded to the pan servo
    tilt_rate: float
    pan_us: float        # pulse widths for those rates
    tilt_us: float
    moving: bool         # error outside the deadband, servos driving
    servo_mode: int      # 0 off (no pulses), 1 automatic, 2 test

    @property
    def valid(self):
        return self.freq > 0


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise SystemExit("Board not found (no /dev/cu.usbmodem*). Is it plugged in?")
    return ports[0]


class Stream:
    """Background reader. Calls from its thread, per packet:
    on_raw(a, b): int16 arrays (HOP, 2) per bus, columns [left, right]
    on_audio(block): (4, HOP) float, mic 1..4, full-scale units
    on_result(r): Result
    """

    def __init__(self, on_raw=None, on_audio=None, on_result=None, port=None):
        self.on_raw, self.on_audio, self.on_result = on_raw, on_audio, on_result
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
        """Servo command to the board: E0/E1 off/auto, TP<r>/TT<r> test rad/s, NP<us>/NT<us> stop pulse, S stop."""
        self.ser.write((command + "\n").encode())

    def close(self):
        self.on_raw = self.on_audio = self.on_result = None
        self.ser.close()

    def _run(self):
        rx = b""
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
                if rx[i + 4 + 2 * BLK16 : i + 8 + 2 * BLK16] != b"TDOA":   # not a real packet start
                    rx = rx[i + 1 :]
                    continue
                pkt = rx[i : i + PKT]
                rx = rx[i + PKT :]
                self.packets += 1
                w = np.frombuffer(pkt[4 : 4 + 2 * BLK16], dtype="<i2")
                a, b = w[: 2 * HOP].reshape(-1, 2), w[2 * HOP :].reshape(-1, 2)
                cb = self.on_raw
                if cb:
                    cb(a, b)
                cb = self.on_audio
                if cb:
                    slots = np.concatenate([a, b], axis=1)              # columns: A left, A right, B left, B right
                    cb(slots[:, MIC_SLOT].T / FULL_SCALE)
                cb = self.on_result
                if cb:
                    v = np.array(struct.unpack(f"<{NF}f", pkt[-4 * NF :]))
                    if np.all(np.isfinite(v)):
                        cb(Result(v[0:3], v[3], v[4], v[5], v[6], bool(v[7]), v[8], v[9], v[10:16], int(v[16]), v[17], v[18],
                                  int(v[19]), v[20:23], v[23:26], v[26], v[27], v[28], v[29], bool(v[30]), int(v[31])))
