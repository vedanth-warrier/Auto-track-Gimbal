"""Servo control window for src/servo_ctl.cpp (env servo_ctl). Two FS90MR: servo 1 on GPIO10, servo 2 on GPIO11.

  uv run host/servo_gui.py

Pick the servo with the SERVO 1 / SERVO 2 buttons (switching stops both). Hold FORWARD or REVERSE to run the
selected servo; let go to stop. Speed = pulse offset from the 1500 us neutral (0-800 us), set with the slider or
typed into the box (Enter). The speed can be changed while holding. Only one servo runs at a time.
The board stops both servos by itself if it hears nothing for 0.3 s, so closing this window always stops them.
"""
import glob
import time
import serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider, TextBox

MAX_US = 800
START_US = 200
REPEAT_S = 0.1                  # resend interval while held (board watchdog: 0.3 s)
SERVO_PINS = {1: 10, 2: 11}     # servo number -> GPIO (must match PINS in servo_ctl.cpp)
FORWARD_SIGN = {1: +1, 2: +1}   # per servo, +1: forward = pulse above neutral. Flip to -1 if FORWARD turns the wrong way
SEL_ON, SEL_OFF = "#f2c66d", "#e6e6e6"

ports = sorted(glob.glob("/dev/cu.usbmodem*"))
if not ports:
    raise SystemExit("ESP32 not found (/dev/cu.usbmodem*). Plugged in? Another program using it?")
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = ports[0], 115200, 0
ser.dtr = False        # DTR/RTS drive the chip's reset/boot lines: keep them low
ser.rts = False
ser.open()

held = None            # +1 forward, -1 reverse, None released
selected = 1           # servo being controlled
last_sent = [0.0, None]
last_reply = [time.time(), None, None]   # time, servo, pulse us
rx = bytearray()


def send(cmd):
    ser.write((cmd + "\n").encode())
    last_sent[:] = [time.time(), cmd]


def command():
    if held is None:
        return "S"
    letter = "F" if held * FORWARD_SIGN[selected] > 0 else "R"
    return f"{letter}{selected} {int(round(speed.val))}"


def set_held(direction):
    global held
    held = direction
    send(command())


fig = plt.figure("Servo control", figsize=(6, 4))
ax_sel = {n: fig.add_axes([0.06 + 0.48 * (n - 1), 0.80, 0.40, 0.13]) for n in SERVO_PINS}
ax_fwd = fig.add_axes([0.06, 0.48, 0.40, 0.25])
ax_rev = fig.add_axes([0.54, 0.48, 0.40, 0.25])
ax_speed = fig.add_axes([0.18, 0.30, 0.50, 0.07])
ax_box = fig.add_axes([0.80, 0.285, 0.14, 0.10])
status = fig.text(0.06, 0.06, "", fontsize=10, family="monospace")
b_sel = {n: Button(ax_sel[n], f"SERVO {n}\nGPIO{pin}") for n, pin in SERVO_PINS.items()}
b_fwd = Button(ax_fwd, "FORWARD\n(hold)", color="#cfe8cf", hovercolor="#9fd49f")
b_rev = Button(ax_rev, "REVERSE\n(hold)", color="#cfd8ef", hovercolor="#9fb4e6")
speed = Slider(ax_speed, "speed (us)", 0, MAX_US, valinit=START_US, valstep=1)
box = TextBox(ax_box, "", initial=str(START_US))
syncing = [False]


def select(n):
    global selected, held
    selected, held = n, None
    send("S")
    for k, b in b_sel.items():
        b.color = b.hovercolor = SEL_ON if k == n else SEL_OFF
        b.ax.set_facecolor(b.color)


def on_slider(v):
    if syncing[0]:
        return
    syncing[0] = True
    box.set_val(str(int(v)))
    syncing[0] = False


def on_box(text):
    if syncing[0]:
        return
    try:
        v = min(max(int(float(text)), 0), MAX_US)
    except ValueError:
        v = int(speed.val)
    syncing[0] = True
    speed.set_val(v)
    box.set_val(str(v))
    syncing[0] = False


speed.on_changed(on_slider)
box.on_submit(on_box)


def on_press(ev):
    if ev.inaxes is ax_fwd:
        set_held(+1)
    elif ev.inaxes is ax_rev:
        set_held(-1)
    for n, ax in ax_sel.items():
        if ev.inaxes is ax:
            select(n)


def on_release(ev):
    if held is not None:
        set_held(None)


def on_leave(ev):              # mouse left the window while holding: stop
    if held is not None:
        set_held(None)


def tick():
    if held is not None and time.time() - last_sent[0] >= REPEAT_S:
        send(command())
    n = ser.in_waiting
    if n:
        rx.extend(ser.read(n))
        lines = rx.split(b"\n")
        rx[:] = lines[-1][-200:]
        for ln in lines[:-1]:
            if ln.startswith(b"OK "):
                try:
                    servo, us = map(int, ln.split()[1:3])
                except ValueError:
                    continue
                last_reply[:] = [time.time(), servo, us]
    servo, us = last_reply[1], last_reply[2]
    if last_sent[1] is not None and time.time() - last_reply[0] > 0.5 and last_sent[0] > last_reply[0]:
        board = "NO REPLY: is servo_ctl flashed?"
    elif us is None:
        board = "connected, servos stopped"
    elif us == 0:
        board = "stopped (no pulses)"
    else:
        way = "forward" if (us - 1500) * FORWARD_SIGN.get(servo, 1) > 0 else "reverse"
        board = f"servo {servo} (GPIO{SERVO_PINS.get(servo, '?')}) {us} us ({way}, {abs(us - 1500)} us)"
    status.set_text(f"{ports[0]}   controlling servo {selected} (GPIO{SERVO_PINS[selected]})\nboard: {board}")
    fig.canvas.draw_idle()


def on_close(ev):
    try:
        send("S")
        time.sleep(0.05)
    finally:
        ser.close()


fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("figure_leave_event", on_leave)
fig.canvas.mpl_connect("close_event", on_close)
timer = fig.canvas.new_timer(interval=50)
timer.add_callback(tick)
timer.start()
select(1)
plt.show()
