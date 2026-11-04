import math
import threading
import time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from stream import Stream, RATE, HOP

AZ_DEG = [90, 180, 0, 270]   # must match AZ_DEG in src/mic.cpp
WEIGHTS = [0.95, 0.88, 1.12, 1.0]   # mic 1..4: multiplies each mic's level before the vector is computed (1 = as the board sends it)
YBOT, YTOP = -110, 0         # dBFS range of the level bars
GATE_DB = -70                # vector drawn grey when the loudest mic is below this
TRAIL_S = 1.0                # seconds of vector history drawn as a trail
SCALE = 5.0                  # display only: drawn vector length = SCALE x true |v|
TAU = 0.15                   # smoothing time constant (s). Lag angle ~ source angular speed x TAU:
                             # a source moving at 1 rad/s trails by ~0.15 rad (~9 deg)
PKT_S = HOP / RATE           # time per packet (8 ms); smoothing steps once per packet
R_MAX = 1.15                 # drawn vector is clipped to this radius (axes are +/-1.2); tip turns red when clipped

latest = None                # (amps[4], x, y, freq, snr) from the newest packet
smooth = None                # [x, y] smoothed vector; held while no tone is detected
trail = deque()              # (time, x, y) of the smoothed vector, tone packets only
lock = threading.Lock()
alpha = 1 - math.exp(-PKT_S / TAU)   # exponential average weight per packet

W = np.array(WEIGHTS)
DX, DY = np.cos(np.radians(AZ_DEG)), np.sin(np.radians(AZ_DEG))

def on_dir(amps, x, y, freq, snr):
    # Recompute the vector from the board's per-mic levels with WEIGHTS applied
    # (with all weights 1 this is exactly the board's own x, y)
    global latest, smooth
    amps = amps * W
    if snr > 0 and amps.sum() > 0:
        x, y = (amps * DX).sum() / amps.sum(), (amps * DY).sum() / amps.sum()
    with lock:
        latest = (amps, x, y, freq, snr)
        if snr > 0:
            if smooth is None:
                smooth = [x, y]
            else:
                smooth[0] += alpha * (x - smooth[0])
                smooth[1] += alpha * (y - smooth[1])
            trail.append((time.time(), smooth[0], smooth[1]))

Stream(on_dir=on_dir)

fig, (ax_bar, ax_vec) = plt.subplots(1, 2, figsize=(10, 5), gridspec_kw={"width_ratios": [1, 1.3]})
colours = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

bars = ax_bar.bar([f"Mic {i + 1}" for i in range(4)], [0] * 4, bottom=YBOT, color=colours)   # empty until data
ax_bar.set_ylim(YBOT, YTOP)
ax_bar.set_ylabel("Tone RMS (dBFS)")
ax_bar.grid(axis="y", alpha=0.3)

ax_vec.set_aspect("equal")
ax_vec.set_xlim(-1.2, 1.2)
ax_vec.set_ylim(-1.2, 1.2)
ax_vec.add_patch(plt.Circle((0, 0), 1, fill=False, color="0.7"))
ax_vec.axhline(0, color="0.85", linewidth=0.8)
ax_vec.axvline(0, color="0.85", linewidth=0.8)
for i, az in enumerate(AZ_DEG):   # where each mic faces
    mx, my = np.cos(np.radians(az)), np.sin(np.radians(az))
    ax_vec.plot(mx, my, "o", color=colours[i])
    ax_vec.annotate(f"Mic {i + 1}", (mx, my), xytext=(mx * 1.12, my * 1.12), ha="center", va="center", fontsize=8)
(trail_ln,) = ax_vec.plot([], [], color="0.6", linewidth=0.8)
(raw_ln,) = ax_vec.plot([0, 0], [0, 0], color="0.7", linewidth=1)    # raw vector
(vec_ln,) = ax_vec.plot([0, 0], [0, 0], color="k", linewidth=2.5)    # smoothed vector
(tip,) = ax_vec.plot([0], [0], "o", color="k")
ax_vec.set_title("Direction vector", fontsize=10)

def clip(px, py):
    # Keep the drawn point inside the axes; same direction, length capped at R_MAX
    r = math.hypot(px, py)
    if r > R_MAX:
        return px * R_MAX / r, py * R_MAX / r, True
    return px, py, False

def update(_):
    with lock:
        if latest is None:
            return []
        amps, x, y, freq, snr = latest
        sx, sy = smooth if smooth is not None else (0.0, 0.0)
        now = time.time()
        while trail and trail[0][0] < now - TRAIL_S:
            trail.popleft()
        tx = [p[1] * SCALE for p in trail]
        ty = [p[2] * SCALE for p in trail]
    db = 20 * np.log10(np.maximum(amps, 1e-12))
    for bar, d in zip(bars, db):
        bar.set_height(max(d, YBOT) - YBOT)   # bars grow up from YBOT
    colour = "k" if snr > 0 and db.max() >= GATE_DB else "0.75"
    rx_, ry_, _ = clip(x * SCALE, y * SCALE)
    px, py, clipped = clip(sx * SCALE, sy * SCALE)
    raw_ln.set_data([0, rx_], [0, ry_])
    vec_ln.set_data([0, px], [0, py])
    vec_ln.set_color(colour)
    tip.set_data([px], [py])
    tip.set_color("tab:red" if clipped else colour)
    ctx = [clip(a, b)[:2] for a, b in zip(tx, ty)]
    trail_ln.set_data([c[0] for c in ctx], [c[1] for c in ctx])
    if snr > 0:
        ax_vec.set_title(f"tone {freq:.0f} Hz, SNR {snr:.0f} dB  smoothed |v| = {np.hypot(sx, sy):.3f}, "
                         f"angle {np.degrees(np.arctan2(sy, sx)) % 360:.0f} deg  (x{SCALE:g})", fontsize=10)
    else:
        ax_vec.set_title("No tone detected", fontsize=10)
    return []

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
