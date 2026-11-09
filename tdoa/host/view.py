"""Live view of the TDOA tracker.

Left: sky seen from above the array. Angle = azimuth (anticlockwise from +x), distance from the centre = angle
off the array axis (0 = straight up the axis). Small grey dot = raw estimate, big dot = tracked direction
(green tracking, orange coasting), red ring = servo command. Dashed circle = search cone (if not full sky).
Right: the 6 pair delays, with the largest possible delay for each pair (its length / speed of sound).
"""
import math
import threading
import time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tstream import Stream
from tconfig import MIC_POS, PAIR_NAMES, PAIR_DIST, SPEED, CONE_DEG

TRAIL_S = 2.0
latest = None
trail = deque()      # (time, az, off-axis) of the tracked direction
lock = threading.Lock()

def on_result(r):
    global latest
    with lock:
        latest = r
        if r.track_state:
            trail.append((time.time(), math.degrees(math.atan2(r.tu[1], r.tu[0])), off_axis(r.tu)))

def off_axis(u):
    return math.degrees(math.acos(max(-1.0, min(1.0, u[2]))))

Stream(on_result=on_result)

fig = plt.figure(figsize=(11, 5))
ax = fig.add_subplot(1, 2, 1, projection="polar")
ax.set_rlim(0, 90)
ax.set_rticks([20, 45, 70, 90])
ax.set_yticklabels(["20", "45", "70", "90 off axis"], fontsize=7)
th = np.linspace(0, 2 * np.pi, 200)
if CONE_DEG < 90:
    ax.plot(th, np.full_like(th, CONE_DEG), "--", color="0.6", linewidth=0.8)
for i, p in enumerate(MIC_POS):
    a = math.atan2(p[1], p[0])
    ax.plot([a], [86], "o", color=f"C{i}")
    ax.annotate(f"Mic {i + 1}", (a, 86), xytext=(a, 98), ha="center", va="center", fontsize=8, annotation_clip=False)
(trail_ln,) = ax.plot([], [], color="0.6", linewidth=0.8)
(raw,) = ax.plot([], [], "o", markersize=5, color="0.6")
(dot,) = ax.plot([], [], "o", markersize=12, color="tab:green")
(cmd,) = ax.plot([], [], "o", markersize=18, markerfacecolor="none", markeredgecolor="tab:red", markeredgewidth=2)
(truth,) = ax.plot([], [], "x", markersize=10, color="tab:red")
title = ax.set_title("No tone", fontsize=10, pad=18)

bx = fig.add_subplot(1, 2, 2)
lim = 1e6 * PAIR_DIST / SPEED
bars = bx.bar(PAIR_NAMES, np.zeros(6), color="tab:blue")
bx.scatter(range(6), lim, marker="_", s=300, color="0.4", label="max possible")
bx.scatter(range(6), -lim, marker="_", s=300, color="0.4")
bx.set_ylim(-1.2 * lim.max(), 1.2 * lim.max())
bx.axhline(0, color="0.8", linewidth=0.8)
bx.set_ylabel("Pair delay t_i - t_j (us)")
bx.set_xlabel("Mic pair i-j")
bx.legend(loc="upper right", fontsize=8)
info = bx.text(0.02, 0.98, "", transform=bx.transAxes, va="top", fontsize=8, family="monospace")

def update(_):
    with lock:
        r = latest
        now = time.time()
        while trail and trail[0][0] < now - TRAIL_S:
            trail.popleft()
        tr = list(trail)
    if r is None:
        return []
    trail_ln.set_data([math.radians(p[1]) for p in tr], [p[2] for p in tr])
    if r.sim_el:
        truth.set_data([math.radians(r.sim_az)], [90 - r.sim_el])
    if r.valid:
        raw.set_data([math.radians(r.az)], [90 - r.el])
        for b, t in zip(bars, r.tau):
            b.set_height(t)
    else:
        raw.set_data([], [])
    if r.track_state:
        taz = math.degrees(math.atan2(r.tu[1], r.tu[0])) % 360
        dot.set_data([math.radians(taz)], [off_axis(r.tu)])
        dot.set_color("tab:green" if r.track_state == 1 else "tab:orange")
        title.set_text(f"tracked: az {taz:.0f} deg, {off_axis(r.tu):.1f} deg off axis"
                       + ("  (coasting)" if r.track_state == 2 else ""))
    else:
        dot.set_data([], [])
        title.set_text("No track" + ("" if r.valid else " (no tone)"))
    if np.any(r.cmd):
        cmd.set_data([math.atan2(r.cmd[1], r.cmd[0])], [off_axis(r.cmd)])
    mode = {0: "off", 1: "auto", 2: "test"}.get(r.servo_mode, "?")
    info.set_text(f"tone {r.freq:6.0f} Hz  SNR {r.snr:4.0f} dB\nquality   {r.quality:.2f}\n"
                  f"ambiguity {r.ambiguity:.2f}\nre-syncs  {r.resyncs}\n"
                  f"servos {mode}{'  MOVING' if r.moving else '  (in deadband)'}\n"
                  f"pan  {r.pan_rate:+.2f} rad/s {r.pan_us:6.0f} us\ntilt {r.tilt_rate:+.2f} rad/s {r.tilt_us:6.0f} us")
    return []

ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()
