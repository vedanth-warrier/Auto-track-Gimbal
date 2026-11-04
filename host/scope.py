import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from stream import Stream, RATE

WINDOW_S = 1.0
DEC = 16          # draw min/max of every 16 samples (keeps peaks, 16x fewer points)
YFLOOR = 2.5e-4   # smallest y-limit (full-scale units), so silence doesn't zoom into the noise floor
HEADROOM = 1.2    # y-limit = 1.2 x max of the last WINDOW_S

N = int(RATE * WINDOW_S)
ring = np.zeros((4, N))      # row 0..3 = mic 1..4, circular, full-scale units
head = 0                     # next write position in ring
lock = threading.Lock()

def on_audio(block):
    global head
    idx = (head + np.arange(block.shape[1])) % N
    with lock:
        ring[:, idx] = block
        head = (head + block.shape[1]) % N

Stream(on_audio=on_audio)

t = np.repeat((np.arange(N // DEC) - (N // DEC - 1)) * DEC / RATE, 2)   # one x per bucket, twice: min then max
ylim = YFLOOR * HEADROOM

fig, axes = plt.subplots(4, 1, sharex=True, sharey=True)
colours = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
lines = []
for i, ax in enumerate(axes):
    (ln,) = ax.plot(t, np.zeros_like(t), color=colours[i], linewidth=0.8)
    ax.set_title(f"Mic {i + 1}", fontsize=9)
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
    if target > ylim or target < ylim * 0.5:   # hysteresis: only rescale on big changes
        ylim = target
        axes[0].set_ylim(-ylim, ylim)
    return lines

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
