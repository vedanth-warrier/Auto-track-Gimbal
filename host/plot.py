import threading
import time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from stream import Stream, RATE

NFFT = 4096        # FFT length: 128 ms of audio, 7.8 Hz per bin
YBOT = -130        # dBFS, bottom of the plot (below the mic noise floor)
YHEAD = 6          # dB of space above the highest peak
PEAK_S = 1.0       # y-top follows the highest peak over this many seconds

ring = np.zeros((4, NFFT))   # row 0..3 = mic 1..4, circular, full-scale units
head = 0                     # next write position in ring
lock = threading.Lock()

def on_audio(block):
    global head
    idx = (head + np.arange(block.shape[1])) % NFFT
    with lock:
        ring[:, idx] = block
        head = (head + block.shape[1]) % NFFT

Stream(on_audio=on_audio)

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
        snap = np.roll(ring, -head, axis=1)   # oldest -> newest
    snap = snap - snap.mean(axis=1, keepdims=True)   # remove DC offset
    mag = np.abs(np.fft.rfft(snap * win, axis=1)) * scale
    db = 20 * np.log10(mag + 1e-12)
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
