import struct
import threading
import time
from collections import deque
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = "/dev/cu.usbmodem2101"
AZ_DEG = [90, 0, 180, 270]   # must match AZ_DEG in src/mic.cpp
YBOT, YTOP = -110, 0         # dBFS range of the level bars
GATE_DB = -70                # vector drawn grey when the loudest mic is below this
TRAIL_S = 1.0                # seconds of vector history drawn as a trail
SCALE = None                 # display gain for the vector; None = auto (longest |v| in last AUTO_S fills 90% of the circle)
AUTO_S = 3.0                 # seconds of history the auto gain looks at
MAX_GAIN = 50                # auto gain cap, so noise isn't blown up to full length

latest = None                # (amps[4], x, y, f0, harmonics) from the newest DIRV packet
trail = deque()              # (time, x, y), pitched packets only
recent = deque()             # (time, |v|), pitched packets in the last AUTO_S, for the auto gain
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
    # Skips the raw audio, keeps only "DIRV" + 8 little-endian float32 (4 amplitudes, x, y, f0, harmonics used)
    global latest
    rx = b""
    while True:
        rx += ser.read(max(1, ser.in_waiting))
        while True:
            i = rx.find(b"DIRV")
            if i < 0:
                rx = rx[-3:]
                break
            if len(rx) < i + 36:
                rx = rx[i:]
                break
            v = struct.unpack("<8f", rx[i + 4 : i + 36])
            rx = rx[i + 36 :]
            if all(np.isfinite(v)):
                with lock:
                    latest = (v[:4], v[4], v[5], v[6], int(v[7]))
                    if v[7] > 0:
                        trail.append((time.time(), v[4], v[5]))
                        recent.append((time.time(), np.hypot(v[4], v[5])))

threading.Thread(target=reader, daemon=True).start()

fig, (ax_bar, ax_vec) = plt.subplots(1, 2, figsize=(10, 5), gridspec_kw={"width_ratios": [1, 1.3]})
colours = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

bars = ax_bar.bar([f"Mic {i + 1}" for i in range(4)], [YBOT] * 4, bottom=0, color=colours)
ax_bar.set_ylim(YBOT, YTOP)
ax_bar.set_ylabel("Harmonic RMS (dBFS)")
ax_bar.grid(axis="y", alpha=0.3)

ax_vec.set_aspect("equal")
ax_vec.set_xlim(-1.2, 1.2)
ax_vec.set_ylim(-1.2, 1.2)
ax_vec.add_patch(plt.Circle((0, 0), 1, fill=False, color="0.7"))
ax_vec.axhline(0, color="0.85", linewidth=0.8)
ax_vec.axvline(0, color="0.85", linewidth=0.8)
for i, az in enumerate(AZ_DEG):   # where each mic faces
    x, y = np.cos(np.radians(az)), np.sin(np.radians(az))
    ax_vec.plot(x, y, "o", color=colours[i])
    ax_vec.annotate(f"Mic {i + 1}", (x, y), xytext=(x * 1.12, y * 1.12), ha="center", va="center", fontsize=8)
(trail_ln,) = ax_vec.plot([], [], color="0.6", linewidth=0.8)
(vec_ln,) = ax_vec.plot([0, 0], [0, 0], color="k", linewidth=2.5)
(tip,) = ax_vec.plot([0], [0], "o", color="k")
ax_vec.set_title("Direction vector", fontsize=10)

def update(_):
    with lock:
        if latest is None:
            return []
        amps, x, y, f0, nh = latest
        now = time.time()
        while trail and trail[0][0] < now - TRAIL_S:
            trail.popleft()
        while recent and recent[0][0] < now - AUTO_S:
            recent.popleft()
        tx = np.array([p[1] for p in trail])
        ty = np.array([p[2] for p in trail])
        vmax = max((p[1] for p in recent), default=0)
    if SCALE is not None:
        gain = SCALE
    else:
        gain = min(MAX_GAIN, 0.9 / vmax) if vmax > 0 else 1.0
        gain = max(gain, 1.0)                  # never shrink below true size
    db = 20 * np.log10(np.maximum(amps, 1e-12))
    for bar, d in zip(bars, db):
        bar.set_y(YBOT)                        # bars grow up from the bottom of the axis
        bar.set_height(max(d, YBOT) - YBOT)
    colour = "k" if nh > 0 and db.max() >= GATE_DB else "0.75"
    vec_ln.set_data([0, x * gain], [0, y * gain])
    vec_ln.set_color(colour)
    tip.set_data([x * gain], [y * gain])
    tip.set_color(colour)
    trail_ln.set_data(tx * gain, ty * gain)
    if nh > 0:
        ax_vec.set_title(f"f0 {f0:.0f} Hz, {nh} harmonics  |v| = {np.hypot(x, y):.3f}  (drawn x{gain:.0f})", fontsize=10)
    else:
        ax_vec.set_title("No pitch in band", fontsize=10)
    return []

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
