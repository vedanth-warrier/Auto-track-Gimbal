import threading
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
MIN_HZ = 100       # ignore bins below this (rumble, DC drift)
HIST_S = 10.0      # seconds of history shown

HOP = BLK // 8               # new samples per mic per packet (256)
H = int(HIST_S * RATE / HOP) # one FFT per packet -> 62.5 points per second

ring = np.zeros((4, NFFT), dtype=np.int32)   # row 0..3 = mic 1..4, circular
head = 0                                     # next write position in ring
hist_amp = np.full((4, H), np.nan)           # dBFS of the loudest bin, per mic, circular
hist_freq = np.full(H, np.nan)               # frequency of the loudest bin, circular
hidx = 0                                     # next write position in history
lock = threading.Lock()

win = np.hanning(NFFT)
scale = 2 / win.sum()                        # bin magnitude -> sine amplitude
freqs = np.fft.rfftfreq(NFFT, 1 / RATE)
lo = np.searchsorted(freqs, MIN_HZ)

# DTR/RTS drive the ESP32-S3's reset/boot lines over USB-Serial-JTAG; keep them low
ser = serial.Serial()
ser.port = PORT
ser.timeout = 0.1
ser.dtr = False
ser.rts = False
ser.open()
ser.reset_input_buffer()

def analyse():
    # FFT the latest NFFT samples, pick the loudest bin across all 4 mics, log each mic's level there
    global hidx
    with lock:
        snap = np.roll(ring, -head, axis=1).astype(np.float64)   # oldest -> newest
    snap -= snap.mean(axis=1, keepdims=True)   # remove DC offset
    mag = np.abs(np.fft.rfft(snap * win, axis=1)) * scale
    k = lo + np.argmax((mag[:, lo:] ** 2).sum(axis=0))
    with lock:
        hist_amp[:, hidx] = 20 * np.log10(mag[:, k] / FULL_SCALE + 1e-12)
        hist_freq[hidx] = freqs[k]
        hidx = (hidx + 1) % H

def reader():
    # Runs in the background: drains the port at full rate and analyses every packet
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
            analyse()

threading.Thread(target=reader, daemon=True).start()

t = np.linspace(-HIST_S, 0, H)
fig, (ax_amp, ax_f) = plt.subplots(2, 1, sharex=True, gridspec_kw={"height_ratios": [3, 1]})
colours = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
lines = []
for i in range(4):
    (ln,) = ax_amp.plot(t, hist_amp[i], color=colours[i], linewidth=0.8, label=f"Mic {i + 1}")
    lines.append(ln)
(fline,) = ax_f.plot(t, hist_freq, color="k", linewidth=0.8)
ax_amp.set_ylabel("Amplitude (dBFS)")
ax_amp.grid(alpha=0.3)
ax_amp.legend(loc="upper left", fontsize=8)
ax_f.set_ylabel("Frequency (Hz)")
ax_f.set_ylim(0, RATE / 2)
ax_f.set_xlim(-HIST_S, 0)
ax_f.set_xlabel("Time (s)")
ax_f.grid(alpha=0.3)

def update(_):
    with lock:
        amp = np.roll(hist_amp, -hidx, axis=1)   # oldest -> newest
        f = np.roll(hist_freq, -hidx)
    for ln, a in zip(lines, amp):
        ln.set_ydata(a)
    fline.set_ydata(f)
    if np.isfinite(amp).any():
        ax_amp.set_ylim(np.floor(np.nanmin(amp) / 10) * 10 - 5, np.ceil(np.nanmax(amp) / 10) * 10 + 5)   # 10 dB steps, less jitter
        ax_amp.set_title(f"Loudest frequency now: {f[-1]:.0f} Hz", fontsize=10)
    return lines + [fline]

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
