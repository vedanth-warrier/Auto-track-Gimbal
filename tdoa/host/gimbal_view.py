"""Live view and control for the gimbal firmware (env gimbal or gimbal_sim).

  uv run python gimbal_view.py

Map of the travel (pan across, elevation up; + pan = FORWARD, towards mic 4): estimated pose (blue dot), where it's
driving to (ring), source direction in the world (orange star; outside the box = out of reach), search stops
(grey ticks). gimbal_sim also shows the true pose and source (grey).
Buttons: AUTO starts tracking (refused until calibrated), OFF stops (no pulses), SET HOME tells the board the
gimbal is at home NOW (pan centred, plate facing the horizon): put it there first, by hand or with the jog
buttons. Hold PAN- / PAN+ / DOWN / UP to jog: counted in the pose, NO limits (watch the wires). Jog speed =
pulse offset from neutral (slider or box + Enter, GIMBAL_CAL_US range), changeable while holding; it's turned
into deg/s with the speed tables in gimbal_config.h, which the board turns back into the same pulse (keep the
flashed firmware and the file in step). Closing the window switches the gimbal off.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider, TextBox
from gstream import GimbalStream, MODES, STATES
import gconfig as G

MIN_US, MAX_US = G.CAL_US[0], G.CAL_US[-1]   # jog speed range (us from neutral)
REPEAT_S = 0.1          # jog resend interval (board stops 0.3 s after the last one)




latest = [None]
st = GimbalStream(on_packet=lambda r, g: latest.__setitem__(0, (r, g)))

fig = plt.figure("Gimbal", figsize=(10, 6))
ax = fig.add_axes([0.07, 0.30, 0.52, 0.62])
ax.set_xlim(G.PAN_MIN - 15, G.PAN_MAX + 15)
ax.set_ylim(G.TILT_MIN - 8, G.TILT_MAX + 8)
ax.set_xlabel("pan (deg), + = FORWARD (towards mic 4)")
ax.set_ylabel("elevation (deg), 0 = horizon")
ax.add_patch(plt.Rectangle((G.PAN_MIN, G.TILT_MIN), G.PAN_MAX - G.PAN_MIN, G.TILT_MAX - G.TILT_MIN, fill=False, lw=1.5))
ax.add_patch(plt.Rectangle((G.P_LO, G.T_LO), G.P_HI - G.P_LO, G.T_HI - G.T_LO, fill=False, ls="--", lw=0.8, color="grey"))
for p in G.search_stops():
    ax.plot(p, G.SEARCH_EL, "|", color="grey", ms=10)
sim_pose, = ax.plot([], [], "o", color="0.7", ms=12, label="true pose (sim)")
sim_src, = ax.plot([], [], "*", color="0.7", ms=16, label="true source (sim)")
target, = ax.plot([], [], "o", mfc="none", mec="tab:blue", ms=16, mew=1.5, label="target")
pose, = ax.plot([], [], "o", color="tab:blue", ms=10, label="estimated pose")
src, = ax.plot([], [], "*", color="tab:orange", ms=16, label="source")
ax.legend(loc="upper left", fontsize=8)
ax.grid(alpha=0.3)
status = fig.text(0.63, 0.92, "", family="monospace", fontsize=9, va="top")
warn = fig.text(0.07, 0.95, "", color="tab:red", fontsize=10, weight="bold")

b_auto = Button(fig.add_axes([0.07, 0.13, 0.14, 0.08]), "AUTO", color="#cfe8cf", hovercolor="#9fd49f")
b_off = Button(fig.add_axes([0.23, 0.13, 0.14, 0.08]), "OFF", color="#f2c6c6", hovercolor="#e69f9f")
b_home = Button(fig.add_axes([0.39, 0.13, 0.20, 0.08]), "SET HOME\n(gimbal is at home now)")
jog_axes = {
    "PAN-": (fig.add_axes([0.07, 0.03, 0.12, 0.07]), "JP", -1, "PAN_REV_DPS"),
    "PAN+": (fig.add_axes([0.21, 0.03, 0.12, 0.07]), "JP", +1, "PAN_FWD_DPS"),
    "DOWN": (fig.add_axes([0.35, 0.03, 0.11, 0.07]), "JT", -1, "TILT_REV_DPS"),
    "UP": (fig.add_axes([0.48, 0.03, 0.11, 0.07]), "JT", +1, "TILT_FWD_DPS"),
}
jog_buttons = [Button(a, name) for name, (a, _, _, _) in jog_axes.items()]
speed = Slider(fig.add_axes([0.73, 0.15, 0.15, 0.04]), "jog (us)", MIN_US, MAX_US, valinit=MIN_US, valstep=1)
speed.valtext.set_visible(False)
box = TextBox(fig.add_axes([0.90, 0.14, 0.07, 0.06]), "", initial=f"{MIN_US:.0f}")
speed_info = fig.text(0.63, 0.04, "", family="monospace", fontsize=9)
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
        v = min(max(float(text), MIN_US), MAX_US)
    except ValueError:
        v = speed.val
    syncing[0] = True
    speed.set_val(round(v))
    box.set_val(f"{round(v):.0f}")
    syncing[0] = False


speed.on_changed(on_slider)
box.on_submit(on_box)


def jog_cmd(cmd, sign, key):
    return f"{cmd}{sign * G.dps_for(speed.val, key):+.4f}"  # flat table regions need the decimals


held = [None]       # (command, sign, table) while a jog button is held
last_jog = [0.0]
hold_t = [0.0]      # when the current jog started
drops = [0]         # checks during a held jog where the board wasn't jogging (resends too slow)


def on_press(ev):
    if ev.inaxes is b_auto.ax:
        st.send("E1")
    elif ev.inaxes is b_off.ax:
        st.send("E0")
    elif ev.inaxes is b_home.ax:
        st.send("H")
    for name, (a, cmd, sign, key) in jog_axes.items():
        if ev.inaxes is a:
            held[0] = (cmd, sign, key)
            st.send(jog_cmd(cmd, sign, key))
            last_jog[0] = hold_t[0] = time.time()


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
    if held[0] is not None and time.time() - last_jog[0] >= REPEAT_S:
        st.send(jog_cmd(*held[0]))
        last_jog[0] = time.time()
    now = time.time()
    if now - rate[0] >= 1:
        rate[2] = (st.packets - rate[1]) / (now - rate[0])
        rate[0], rate[1] = now, st.packets
    speed_info.set_text(f"jog at {speed.val:.0f} us (deg/s):\n"
                        f"pan  +{G.dps_for(speed.val, 'PAN_FWD_DPS'):5.0f}  -{G.dps_for(speed.val, 'PAN_REV_DPS'):5.0f}\n"
                        f"elev up {G.dps_for(speed.val, 'TILT_FWD_DPS'):4.0f}  down {G.dps_for(speed.val, 'TILT_REV_DPS'):4.0f}")
    x = latest[0]
    if x is None:
        status.set_text("waiting for packets...\n(is the gimbal firmware flashed?)")
        fig.canvas.draw_idle()
        return
    r, g = x
    sim = g.sim_t > 0
    if held[0] is not None and time.time() - hold_t[0] > 0.5 and g.mode != 2:
        drops[0] += 1
    pose.set_data([g.pan], [g.tilt])
    target.set_data([g.tgt_pan], [g.tgt_tilt])
    if g.state == 1:   # the board only works out the world direction while tracking
        src.set_data([g.src_az], [g.src_el])
    else:
        src.set_data([], [])
    if sim:
        sim_pose.set_data([g.sim_pan], [g.sim_tilt])
        sim_src.set_data(*(([g.sim_src_az], [g.sim_src_el]) if g.sim_tone else ([], [])))
    lim = [n for b, n in ((1, "pan min"), (2, "pan max"), (4, "elev min"), (8, "elev max")) if g.limits & b]
    tone = f"{r.freq:7.1f} Hz  SNR {r.snr:4.1f} dB  q {r.quality:.2f}" if r.valid else "none"
    lines = [
        f"mode    {MODES[g.mode]}{'  (SIM)' if sim else ''}",
        f"state   {STATES[g.state]}" + (f"  pass {g.search_pass}" if g.state in (3, 4) else ""),
        f"tone    {tone}",
        f"track   {['lost', 'tracking', 'coasting'][r.track_state]}",
        "",
        f"pose    pan {g.pan:6.1f}  elev {g.tilt:5.1f}  (estimate)",
        f"target  pan {g.tgt_pan:6.1f}  elev {g.tgt_tilt:5.1f}",
        f"error   pan {g.err_pan:6.1f}  elev {g.err_tilt:5.1f}  deg",
        f"speed   pan {g.pan_rate:6.1f}  elev {g.tilt_rate:5.1f}  deg/s",
        f"pulse   pan {g.pan_us:6.0f}  elev {g.tilt_us:5.0f}  us",
        f"source  az {g.src_az:6.1f}  el {g.src_el:5.1f}  (world)",
        f"limits  {', '.join(lim) if lim else '-'}",
        f"slack   elev {g.tilt_slack:4.1f} deg left before it moves down" if G.TILT_BACKLASH > 0 else "slack   elev not set",
        "",
        f"packets {rate[2]:5.1f}/s   dropped by board {g.tx_drops}",
        f"jog     drop-outs while held: {drops[0]}" + ("  (resends too slow: estimate unreliable)" if drops[0] else ""),
    ]
    if sim:
        lines += [f"sim     t {g.sim_t:5.1f}  true pan {g.sim_pan:6.1f}  elev {g.sim_tilt:5.1f}",
                  f"        drift pan {g.pan - g.sim_pan:+5.1f}  elev {g.tilt - g.sim_tilt:+5.1f}"]
    status.set_text("\n".join(lines))
    warn.set_text("" if g.calibrated or sim else "NOT CALIBRATED: AUTO is refused. Run gimbal_cal.py --write, reflash.")
    fig.canvas.draw_idle()


timer = fig.canvas.new_timer(interval=100)
timer.add_callback(tick)
timer.start()
plt.show()
