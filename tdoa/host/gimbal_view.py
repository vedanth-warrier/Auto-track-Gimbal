"""Live view and control for the gimbal firmware (env gimbal or gimbal_sim).

  uv run python gimbal_view.py

Left: where the beacon is relative to the plate axis, from the mics (centre = dead ahead; x = pan error, + = towards
mic 4; y = elevation error, + = up), with the last 2 s as a trail. Inside the inner circle (GIMBAL_STOP_DEG) the
gimbal stops; it starts again outside the outer one (GIMBAL_START_DEG).
Buttons: AUTO starts tracking, OFF stops (no pulses). Hold PAN- / PAN+ / DOWN / UP to jog at the slider's pulse
offset (us from neutral), changeable while holding. Closing the window switches the gimbal off.
"""
import time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider, TextBox
from gstream import GimbalStream, MODES, STATES
import gconfig as G

REPEAT_S = 0.1          # jog resend interval (board stops 0.3 s after the last one)
JOG_MIN, JOG_MAX = min(G.PAN_MIN_US, G.TILT_MIN_US), 800
TRAIL_S = 2.0
LIM = 90

latest = [None]
trail = deque()         # (time, pan error, elevation error) while tracking


def on_packet(r, g):
    latest[0] = (r, g)
    if g.state == 1:
        trail.append((time.time(), g.err_pan, g.err_tilt))


st = GimbalStream(on_packet=on_packet)

fig = plt.figure("Gimbal", figsize=(10, 6))
ax = fig.add_axes([0.07, 0.30, 0.48, 0.62])
ax.set_xlim(-LIM, LIM)
ax.set_ylim(-LIM, LIM)
ax.set_aspect("equal")
ax.set_xlabel("pan error (deg), + = towards mic 4 (FORWARD)")
ax.set_ylabel("elevation error (deg), + = up")
ax.axhline(0, color="0.8", lw=0.8)
ax.axvline(0, color="0.8", lw=0.8)
th = np.linspace(0, 2 * np.pi, 200)
ax.plot(G.STOP * np.cos(th), G.STOP * np.sin(th), color="tab:green", lw=1, label=f"stop inside {G.STOP:.0f}")
if G.START != G.STOP:
    ax.plot(G.START * np.cos(th), G.START * np.sin(th), "--", color="tab:green", lw=1, label=f"start outside {G.START:.0f}")
trail_line, = ax.plot([], [], "-", color="tab:orange", alpha=0.4)
dot, = ax.plot([], [], "o", color="tab:orange", ms=12, label="beacon")
ax.legend(loc="upper left", fontsize=8)
ax.grid(alpha=0.3)
status = fig.text(0.60, 0.92, "", family="monospace", fontsize=9, va="top")

b_auto = Button(fig.add_axes([0.07, 0.13, 0.20, 0.08]), "AUTO", color="#cfe8cf", hovercolor="#9fd49f")
b_off = Button(fig.add_axes([0.30, 0.13, 0.20, 0.08]), "OFF", color="#f2c6c6", hovercolor="#e69f9f")
jog_axes = {
    "PAN-": (fig.add_axes([0.07, 0.03, 0.11, 0.07]), "JP", -1),
    "PAN+": (fig.add_axes([0.20, 0.03, 0.11, 0.07]), "JP", +1),
    "DOWN": (fig.add_axes([0.33, 0.03, 0.10, 0.07]), "JT", -1),
    "UP": (fig.add_axes([0.45, 0.03, 0.10, 0.07]), "JT", +1),
}
jog_buttons = [Button(a, name) for name, (a, _, _) in jog_axes.items()]
speed = Slider(fig.add_axes([0.70, 0.15, 0.17, 0.04]), "jog (us)", JOG_MIN, JOG_MAX, valinit=JOG_MIN, valstep=1)
speed.valtext.set_visible(False)
box = TextBox(fig.add_axes([0.90, 0.14, 0.07, 0.06]), "", initial=f"{JOG_MIN:.0f}")
syncing = [False]


def on_slider(v):
    if syncing[0]:
        return
    syncing[0] = True
    box.set_val(f"{v:.0f}")
    syncing[0] = False


def on_box(text):
    if syncing[0]:
        return
    try:
        v = min(max(float(text), JOG_MIN), JOG_MAX)
    except ValueError:
        v = speed.val
    syncing[0] = True
    speed.set_val(round(v))
    box.set_val(f"{round(v):.0f}")
    syncing[0] = False


speed.on_changed(on_slider)
box.on_submit(on_box)
held = [None]           # (command, sign) while a jog button is held
last_jog = [0.0]


def jog_cmd(cmd, sign):
    return f"{cmd}{sign * speed.val:+.0f}"


def on_press(ev):
    if ev.inaxes is b_auto.ax:
        st.send("E1")
    elif ev.inaxes is b_off.ax:
        st.send("E0")
    for name, (a, cmd, sign) in jog_axes.items():
        if ev.inaxes is a:
            held[0] = (cmd, sign)
            st.send(jog_cmd(cmd, sign))
            last_jog[0] = time.time()


def on_release(ev):
    if held[0] is not None:
        held[0] = None
        st.send("E0")


fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("figure_leave_event", on_release)
fig.canvas.mpl_connect("close_event", lambda ev: (st.send("E0"), time.sleep(0.05), st.close()))
rate = [time.time(), 0, 0.0]


def tick():
    now = time.time()
    if held[0] is not None and now - last_jog[0] >= REPEAT_S:
        st.send(jog_cmd(*held[0]))
        last_jog[0] = now
    if now - rate[0] >= 1:
        rate[2] = (st.packets - rate[1]) / (now - rate[0])
        rate[0], rate[1] = now, st.packets
    while trail and now - trail[0][0] > TRAIL_S:
        trail.popleft()
    x = latest[0]
    if x is None:
        status.set_text("waiting for packets...\n(is the gimbal firmware flashed?)")
        fig.canvas.draw_idle()
        return
    r, g = x
    sim = g.sim_t > 0
    pts = list(trail)
    trail_line.set_data([p[1] for p in pts], [p[2] for p in pts])
    if g.state == 1:
        dot.set_data([np.clip(g.err_pan, -LIM, LIM)], [np.clip(g.err_tilt, -LIM, LIM)])
    else:
        dot.set_data([], [])
    blk = [n for b, n in ((1, "pan FORWARD"), (2, "pan REVERSE"), (4, "elev UP"), (8, "elev DOWN")) if g.blocked & b]
    tone = f"{r.freq:7.1f} Hz  SNR {r.snr:4.1f} dB  q {r.quality:.2f}" if r.valid else "none"
    lines = [
        f"mode    {MODES[g.mode]}{'  (SIM)' if sim else ''}",
        f"state   {STATES[g.state]}" + ("  moving" if g.moving else ("  on target" if g.state == 1 else "")),
        f"tone    {tone}",
        f"track   {['lost', 'tracking', 'coasting'][r.track_state]}" + ("" if g.listening else "  (frozen: servos noisy)"),
        "",
        f"error   pan {g.err_pan:6.1f}  elev {g.err_tilt:6.1f}  total {g.err:5.1f} deg",
        f"pulse   pan {g.pan_us:6.0f}  elev {g.tilt_us:6.0f} us",
        f"stops   {', '.join(blk) if blk else '-'}  (blocked: pushing without the error shrinking)",
        "",
        f"packets {rate[2]:5.1f}/s   dropped by board {g.tx_drops}",
    ]
    if sim:
        a, e = np.radians([g.sim_pan, g.sim_tilt])
        sa, se = np.radians([g.sim_src_az, g.sim_src_el])
        true_err = np.degrees(np.arccos(np.clip(np.cos(e) * np.cos(a) * np.cos(se) * np.cos(sa) + np.cos(e) * np.sin(a)
                                                * np.cos(se) * np.sin(sa) + np.sin(e) * np.sin(se), -1, 1)))
        lines += [f"sim     t {g.sim_t:5.1f}  true pan {g.sim_pan:6.1f}  elev {g.sim_tilt:5.1f}",
                  f"        source az {g.sim_src_az:6.1f}  el {g.sim_src_el:5.1f}  {'on' if g.sim_tone else 'off'}",
                  f"        true error {true_err:5.1f} deg   pushed on stops {g.sim_push:5.1f} s"]
    status.set_text("\n".join(lines))
    fig.canvas.draw_idle()


timer = fig.canvas.new_timer(interval=100)
timer.add_callback(tick)
timer.start()
plt.show()
