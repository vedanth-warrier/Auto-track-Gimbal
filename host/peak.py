import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from stream import Stream, RATE, HOP

NFFT = 4096        # FFT length: 128 ms of audio, 7.8 Hz per bin
MIN_HZ = 100       # ignore bins below this (rumble, DC drift)
HIST_S = 10.0      # seconds of history shown

H = int(HIST_S * RATE / HOP)  # one FFT per packet -> 125 points per second

ring = np.zeros((4, NFFT))                   # row 0..3 = mic 1..4, circular, full-scale units
head = 0                                     # next write position in ring
hist_amp = np.full((4, H), np.nan)           # dBFS of the loudest bin, per mic, circular
hist_freq = np.full(H, np.nan)               # frequency of the loudest bin, circular
hidx = 0                                     # next write position in history
lock = threading.Lock()

win = np.hanning(NFFT)
scale = 2 / win.sum()                        # bin magnitude -> sine amplitude
freqs = np.fft.rfftfreq(NFFT, 1 / RATE)
lo = np.searchsorted(freqs, MIN_HZ)

def on_audio(block):
    # Append the packet, FFT the latest NFFT samples, log each mic's level at the loudest bin across all 4
    global head, hidx
    idx = (head + np.arange(block.shape[1])) % NFFT
    with lock:
        ring[:, idx] = block
        head = (head + block.shape[1]) % NFFT
        snap = np.roll(ring, -head, axis=1)   # oldest -> newest
    snap = snap - snap.mean(axis=1, keepdims=True)   # remove DC offset
    mag = np.abs(np.fft.rfft(snap * win, axis=1)) * scale
    k = lo + np.argmax((mag[:, lo:] ** 2).sum(axis=0))
    with lock:
        hist_amp[:, hidx] = 20 * np.log10(mag[:, k] + 1e-12)
        hist_freq[hidx] = freqs[k]
        hidx = (hidx + 1) % H

Stream(on_audio=on_audio)

t = (np.arange(H) - (H - 1)) * HOP / RATE   # one point per packet, newest at 0
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
    if np.isfinite(f[-1]):
        ax_amp.set_title(f"Loudest frequency now: {f[-1]:.0f} Hz", fontsize=10)
    return lines + [fline]

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
