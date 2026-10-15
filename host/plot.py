import threading
import time
from collections import deque
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = "/dev/cu.usbmodem101"
RATE = 16000
SHIFT = 8
BLK = 2048
PKT = 4 + 2*BLK
NFFT = 2048        # FFT length: 128 ms of audio, 7.8 Hz per bin
FULL_SCALE = 2**23 # 24-bit signed full scale -> 0 dBFS
YBOT = -130        # dBFS, bottom of the plot (below the mic noise floor)
YHEAD = 6          # dB of space above the highest peak
PEAK_S = 1.0       # y-top follows the highest peak over this many seconds

ring = np.zeros((4, NFFT), dtype=np.int32)   # row 0..3 = mic 1..4, circular
head = 0                                     # next write position in ring
lock = threading.Lock()

# DTR/RTS drive the ESP32-S3's reset/boot lines over USB-Serial-JTAG; keep them low
ser = serial.Serial()
ser.port = PORT
ser.timeout = 0.1
ser.dtr = False
ser.rts = False
ser.open()
ser.reset_input_buffer()

def reader():
    # Runs in the background so the port is drained at full rate, however slow drawing is
    global head
    rx = b""
    while True:
        rx += ser.read(max(1, ser.in_waiting))
        while True:
            i = rx.find(b"SYNC")
            if i < 0:
                rx = rx[-3:]
                break
            if len(rx) < i + PKT:
                rx = rx[i:]
                break
            body = rx[i + 4 : i + PKT]
            rx = rx[i + PKT :]
            w = np.frombuffer(body, dtype="<i4") >> SHIFT
            a = w[: BLK // 4].reshape(-1, 2)    # bus A: col0 = left = mic 2, col1 = right = mic 1
            b = w[BLK // 4 :].reshape(-1, 2)    # bus B: col0 = left = mic 4, col1 = right = mic 3
            block = np.stack([a[:, 1], a[:, 0], b[:, 1], b[:, 0]])
            idx = (head + np.arange(block.shape[1])) % NFFT
            with lock:
                ring[:, idx] = block
                head = (head + block.shape[1]) % NFFT

threading.Thread(target=reader, daemon=True).start()

win = np.hanning(NFFT)
scale = 2 / win.sum()                          # bin magnitude -> sine amplitude
freqs = np.fft.rfftfreq(NFFT, 1 / RATE)
peaks = deque()                                # (time, peak dBFS) over the last PEAK_S
ytop = YBOT + 40

fig, ax = plt.subplots()
colours = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
lines = []
for i in range(4):
    (ln,) = ax.plot(freqs, np.full_like(freqs, YBOT), color=colours[i], linewidth=0.8, label=f"Mic {i + 1}")
    lines.append(ln)
ax.set_xlim(0, RATE / 2)
ax.set_ylim(YBOT, ytop)
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Amplitude (dBFS)")
ax.grid(alpha=0.3)
ax.legend(loc="upper right", fontsize=8)

def update(_):
    global ytop
    with lock:
        snap = np.roll(ring, -head, axis=1).astype(np.float64)   # oldest -> newest
    snap -= snap.mean(axis=1, keepdims=True)   # remove DC offset
    mag = np.abs(np.fft.rfft(snap * win, axis=1)) * scale
    db = 20 * np.log10(mag / FULL_SCALE + 1e-12)
    for ln, d in zip(lines, db):
        ln.set_ydata(d)

    now = time.time()
    peaks.append((now, db.max()))
    while peaks[0][0] < now - PEAK_S:
        peaks.popleft()
    target = max(p for _, p in peaks) + YHEAD
    if target > ytop or target < ytop - 20:   # hysteresis: only rescale on big changes
        ytop = target
        ax.set_ylim(YBOT, ytop)
    return lines

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
