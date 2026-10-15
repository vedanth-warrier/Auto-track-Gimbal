import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = "/dev/cu.usbmodem101"
RATE = 16000
WINDOW_S = 1.0
YLIM = 500000
SHIFT = 8
BLK = 2048
PKT = 4 + 2*BLK

N = int(RATE * WINDOW_S)
t = np.linspace(-WINDOW_S, 0, N)
chans = [np.zeros(N) for _ in range(4)]   # index 0..3 = mic 1..4
rx = b""

ser = serial.Serial(PORT, 115200, timeout=0)
ser.reset_input_buffer()

fig, axes = plt.subplots(4, 1, sharex=True, sharey=True)
colours = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
lines = []
for i, ax in enumerate(axes):
    (ln,) = ax.plot(t, chans[i], color=colours[i])
    ax.set_title(f"Mic {i + 1}", fontsize=9)
    lines.append(ln)
axes[0].set_ylim(-YLIM, YLIM)
axes[-1].set_xlabel("Time (s)")

def push(buf, new):
    buf[:-len(new)] = buf[len(new):]
    buf[-len(new):] = new

def update(_):
    global rx
    rx += ser.read(ser.in_waiting)
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
        push(chans[0], a[:, 1])
        push(chans[1], a[:, 0])
        push(chans[2], b[:, 1])
        push(chans[3], b[:, 0])
    for ln, c in zip(lines, chans):
        ln.set_ydata(c)
    return lines

ani = animation.FuncAnimation(fig, update, interval=30, blit=True, cache_frame_data=False)
plt.show()