"""Shared reader for the board's serial stream (keep in sync with src/mic.cpp).

Packet, 125 per second: "SYNC" + bus A + bus B (each 256 stereo frames of int16, [left, right])
+ "DIRV" + 8 float32 (4 tone amplitudes, x, y, tone frequency, SNR dB), all little-endian.
Mic 1 = bus A right, mic 2 = bus A left, mic 3 = bus B right, mic 4 = bus B left (tap-tested).
"""
import glob
import struct
import threading
import numpy as np
import serial

RATE = 32000
HOP = 256                      # new samples per mic per packet (8 ms)
BLK16 = HOP * 2 * 2            # bytes per bus per packet
PKT = 4 + 2 * BLK16 + 36       # SYNC + bus A + bus B + DIRV
FULL_SCALE = 32768             # int16 full scale -> 0 dBFS


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise SystemExit("Board not found (no /dev/cu.usbmodem*). Is it plugged in?")
    return ports[0]


class Stream:
    """Reads the port in a background thread and calls, from that thread, for every packet:
    on_audio(block): block is (4, HOP) samples in full-scale units, row 0..3 = mic 1..4
    on_dir(amps, x, y, freq, snr): the board's tone result; snr == 0 means no tone detected
    """

    def __init__(self, on_audio=None, on_dir=None, port=None):
        self.on_audio, self.on_dir = on_audio, on_dir
        self.ser = serial.Serial()
        self.ser.port = port or find_port()
        self.ser.timeout = 0.1
        self.ser.dtr = False   # DTR/RTS drive the ESP32-S3's reset/boot lines over USB-Serial-JTAG
        self.ser.rts = False
        self.ser.open()
        self.ser.reset_input_buffer()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        rx = b""
        while True:
            rx += self.ser.read(max(1, self.ser.in_waiting))
            while True:
                i = rx.find(b"SYNC")
                if i < 0:
                    rx = rx[-3:]
                    break
                if len(rx) < i + PKT:
                    rx = rx[i:]
                    break
                if rx[i + PKT - 36 : i + PKT - 32] != b"DIRV":   # not a real packet start
                    rx = rx[i + 1 :]
                    continue
                pkt = rx[i : i + PKT]
                rx = rx[i + PKT :]
                if self.on_audio:
                    w = np.frombuffer(pkt[4 : 4 + 2 * BLK16], dtype="<i2") / FULL_SCALE
                    a = w[: 2 * HOP].reshape(-1, 2)   # bus A: col0 = left = mic 2, col1 = right = mic 1
                    b = w[2 * HOP :].reshape(-1, 2)   # bus B: col0 = left = mic 4, col1 = right = mic 3
                    self.on_audio(np.stack([a[:, 1], a[:, 0], b[:, 1], b[:, 0]]))
                if self.on_dir:
                    v = struct.unpack("<8f", pkt[-32:])
                    if all(np.isfinite(v)):
                        self.on_dir(np.array(v[:4]), v[4], v[5], v[6], v[7])
