"""4 scrolling waveforms from the TDOA firmware, for tap tests (is each mic on the right channel?).

Tap one mic at a time: its trace jumps, and its title turns red with "LOUDEST" when its sharp-sound
level over the last TAP_S is clearly above the other three.
"""
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tstream import Stream
from tconfig import RATE

WINDOW_S = 1.0
DEC = 16          # draw min/max of every 16 samples
YFLOOR = 2.5e-4   # smallest y-limit (full-scale units)
HEADROOM = 1.2
TAP_S = 0.5       # window for picking the loudest mic
TAP_DB = 6        # loudest must beat the next mic by this much

N = int(RATE * WINDOW_S)
ring = np.zeros((4, N))      # row 0..3 = mic 1..4, circular, full-scale units
head = 0
lock = threading.Lock()

def on_audio(block):
    global head
    idx = (head + np.arange(block.shape[1])) % N
    with lock:
        ring[:, idx] = block
        head = (head + block.shape[1]) % N

Stream(on_audio=on_audio)

t = np.repeat((np.arange(N // DEC) - (N // DEC - 1)) * DEC / RATE, 2)
ylim = YFLOOR * HEADROOM
fig, axes = plt.subplots(4, 1, sharex=True, sharey=True)
colours = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
lines, titles = [], []
for i, ax in enumerate(axes):
    (ln,) = ax.plot(t, np.zeros_like(t), color=colours[i], linewidth=0.8)
    titles.append(ax.set_title(f"Mic {i + 1}", fontsize=9))
    lines.append(ln)
axes[0].set_ylim(-ylim, ylim)
axes[-1].set_xlim(-WINDOW_S, 0)
axes[-1].set_xlabel("Time (s)")
axes[2].set_ylabel("Amplitude (full scale)")

def update(_):
    global ylim
    with lock:
        snap = np.roll(ring, -head, axis=1)   # oldest -> newest
    env = snap.reshape(4, -1, DEC)
    y = np.empty((4, 2 * (N // DEC)))
    y[:, 0::2] = env.min(axis=2)
    y[:, 1::2] = env.max(axis=2)
    for ln, c in zip(lines, y):
        ln.set_ydata(c)
    target = max(np.abs(snap).max(), YFLOOR) * HEADROOM
    if target > ylim or target < ylim * 0.5:
        ylim = target
        axes[0].set_ylim(-ylim, ylim)
    # sharp-sound level per mic (first difference removes rumble) over the last TAP_S
    recent = np.diff(snap[:, -int(TAP_S * RATE):], axis=1)
    lv = 10 * np.log10((recent ** 2).mean(1) + 1e-30)
    order = np.argsort(lv)[::-1]
    for i, tt in enumerate(titles):
        lead = i == order[0] and lv[order[0]] - lv[order[1]] >= TAP_DB
        tt.set_text(f"Mic {i + 1}" + ("   LOUDEST" if lead else ""))
        tt.set_color("tab:red" if lead else "black")
    return lines

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
