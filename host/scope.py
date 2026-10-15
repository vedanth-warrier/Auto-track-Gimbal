import threading
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = "/dev/cu.usbmodem101"
RATE = 16000
WINDOW_S = 1.0
SHIFT = 8
BLK = 2048
PKT = 4 + 2*BLK
DEC = 8           # draw min/max of every 8 samples (keeps peaks, 8x fewer points)
YFLOOR = 2000     # smallest y-limit, so silence doesn't zoom into the noise floor
HEADROOM = 1.2    # y-limit = 1.2 x max of the last WINDOW_S

N = int(RATE * WINDOW_S)
ring = np.zeros((4, N), dtype=np.int32)   # row 0..3 = mic 1..4, circular
head = 0                                  # next write position in ring
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
            idx = (head + np.arange(block.shape[1])) % N
            with lock:
                ring[:, idx] = block
                head = (head + block.shape[1]) % N

threading.Thread(target=reader, daemon=True).start()

t = np.repeat(np.linspace(-WINDOW_S, 0, N // DEC), 2)   # each x twice: min then max
ylim = YFLOOR * HEADROOM

fig, axes = plt.subplots(4, 1, sharex=True, sharey=True)
colours = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
lines = []
for i, ax in enumerate(axes):
    (ln,) = ax.plot(t, np.zeros_like(t), color=colours[i], linewidth=0.8)
    ax.set_title(f"Mic {i + 1}", fontsize=9)
    lines.append(ln)
axes[0].set_ylim(-ylim, ylim)
axes[-1].set_xlabel("Time (s)")

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
    target = max(int(np.abs(snap).max()), YFLOOR) * HEADROOM
    if target > ylim or target < ylim * 0.5:   # hysteresis: only rescale on big changes
        ylim = target
        axes[0].set_ylim(-ylim, ylim)
    return lines

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
