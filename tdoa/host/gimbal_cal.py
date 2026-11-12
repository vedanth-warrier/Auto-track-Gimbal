"""Gimbal calibration with easy angles (no protractor). Env gimbal, plate fitted.

  uv run python gimbal_cal.py [--axis tilt|pan|both] [--write]

Every run goes between two positions you can see without measuring:
  elevation  0 = plate facing the horizon (plate vertical); 90 = plate facing straight up (plate horizontal).
             A phone level helps (iPhone Measure app > Level, or any clinometer app): flat on the plate it reads
             90 at elevation 0 and 0 at elevation 90.
  pan        -90 / +90 = plate facing straight out to one side / the other (+ = towards mic 4), 180 deg apart.
             An A4 sheet under the base, one edge along the centre direction, shows the side directions.

Each run: Enter, and the board moves about 60 % of the way by itself; then fine-adjust in board-timed steps (type
roughly how many degrees are left) until it's exactly on the target. Only step towards the target (going back
would cross the gear slack): if you overshoot, redo the run. The board counts every bit of it, the same way it counts while tracking, so it knows how far it thinks
the gimbal went; the target says how far it really went.

Elevation: UP 0 -> 90 and DOWN 90 -> 0 at each pulse offset in GIMBAL_CAL_US, plus one repeat as a check. The
first run starts from home with the slack taken up for moving up (no slack crossed); every later one starts
straight after a reversal and crosses the slack first. Solved together: speed per offset and direction, and the
slack. Pan: FORWARD -90 -> +90 and REVERSE +90 -> -90 at each offset (any pan slack is absorbed into the speeds).
--write saves the speed tables (and the slack) into src/gimbal_config.h; then reflash.
"""
import argparse
import json
import os
import sys
import threading
import time
import numpy as np
from gstream import GimbalStream
import gconfig as G

STEP = 3.0          # default fine-adjust step, deg
FIRST_FRAC = 0.6    # first move of each run: board-timed, this share of the way by the current speed table
                    # (at 500-800 us a whole run takes ~0.4 s: too fast to stop by hand). Tables up to ~65 %
                    # too slow still stop short of the target
SETTLE_S = 0.3
NEUTRAL = 1500.0    # GIMBAL_*_NEUTRAL_US, for the pulse check

latest = [None]
st = None
last_dir = {"T": +1, "P": +1}   # direction of the last move per axis (elevation: home = slack taken up for UP)
pulse_warned = [False]


def key(axis, sign):
    return ("TILT" if axis == "T" else "PAN") + ("_FWD_DPS" if sign > 0 else "_REV_DPS")


def way(axis, sign):
    return ("UP" if sign > 0 else "DOWN") if axis == "T" else ("FORWARD" if sign > 0 else "REVERSE")


def snapshot():
    time.sleep(SETTLE_S)
    return latest[0]


def counted(axis, a, b):
    """Motor travel the board counted between two packets (deg). Elevation: plate change + change of the slack
    left, which doesn't depend on the slack value flashed."""
    if axis == "P":
        return b.pan - a.pan
    return (b.tilt - a.tilt) + (b.tilt_slack - a.tilt_slack)


def check_pulse(axis, sign, us):
    g = latest[0]
    p = g.tilt_us if axis == "T" else g.pan_us
    if p and abs(p - (NEUTRAL + sign * us)) > 2 and not pulse_warned[0]:
        pulse_warned[0] = True
        print(f"    WARNING: board sends {p:.0f} us, expected {NEUTRAL + sign * us:.0f}: the flashed speed tables differ"
              " from gimbal_config.h. Reflash, then rerun.")


def drive(axis, sign, us, prompt):
    """Counted jog at offset us until Enter."""
    dps = G.dps_for(us, key(axis, sign))
    input(prompt)
    going = [True]

    def keep():
        while going[0]:
            st.send(f"J{axis}{sign * dps:+.4f}")
            time.sleep(0.1)

    th = threading.Thread(target=keep, daemon=True)
    th.start()
    time.sleep(0.25)
    check_pulse(axis, sign, us)
    input("    ...")
    going[0] = False
    st.send("E0")       # stop now, not after the repeat thread's next wake-up (up to 0.1 s = 25 deg at 800 us)
    th.join()
    st.send("E0")       # in case the thread sent one more jog in between
    last_dir[axis] = sign


def step(axis, sign, us, deg):
    """Board-timed counted step of about deg (at offset us)."""
    dps = G.dps_for(us, key(axis, sign))
    ms = max(8.0, deg / dps * 1000)
    st.send(f"N{axis}{sign * dps:+.4f} {ms:.0f}")
    time.sleep(ms / 1000 + 0.15)
    last_dir[axis] = sign


def run_to(axis, sign, us, target, dist):
    """Board-timed first move, then fine-adjust towards target, one direction only. Returns number of segments,
    or None to redo."""
    input(f"    Enter: moves {way(axis, sign)} about {FIRST_FRAC * dist:.0f} of the {dist:.0f} deg (board-timed)")
    step(axis, sign, us, FIRST_FRAC * dist)
    segs = 1
    while True:
        s = input(f"    fine-adjust: Enter = {STEP:.0f} deg step, a number = that many deg, d = exactly at {target}, "
                  "r = redo: ").strip().lower()
        if s == "d":
            return segs
        if s == "r":
            return None     # overshot: the run is redone from the start
        try:
            deg = STEP if s == "" else float(s)
        except ValueError:
            continue
        if deg <= 0:
            print("    only towards the target (redo if it overshot)")
            continue
        step(axis, sign, us, deg)
        segs += 1


def position(axis, target, hand_ok):
    """Free positioning (both ways, slowest offset) until exactly at target."""
    us = G.CAL_US[0]
    names = (way(axis, +1), way(axis, -1))
    print(f"    get it exactly to {target}: f / b = drive {names[0]} / {names[1]} (Enter stops), "
          f"+N / -N = step N deg, d = done" + (", h = set by hand (slack taken up for moving UP)" if hand_ok else ""))
    while True:
        s = input("    > ").strip().lower()
        if s == "d":
            return
        if s == "h" and hand_ok:
            last_dir[axis] = +1
            return
        if s in ("f", "b"):
            drive(axis, +1 if s == "f" else -1, us, "    Enter to start")
            continue
        try:
            deg = float(s)
        except ValueError:
            continue
        if deg:
            step(axis, 1 if deg > 0 else -1, us, abs(deg))


def measure(axis, sign, ui, start, target, n, total, log):
    us = G.CAL_US[ui]
    while True:
        crossed = last_dir[axis] != sign
        print(f"  run {n}/{total}: {way(axis, sign)} {start} -> {target} at {us:.0f} us"
              + ("  (crosses the slack first)" if crossed and axis == "T" else ""))
        a = snapshot()
        segs = run_to(axis, sign, us, target, abs(target - start))
        if segs is None:
            print(f"    redo: put it back exactly at {start}")
            position(axis, start, hand_ok=(axis == "T" and start == 0))
            continue
        b = snapshot()
        m = abs(counted(axis, a, b))
        print(f"    board counted {m:.1f} deg of {'motor ' if axis == 'T' else ''}travel in {segs} segment(s)")
        log.append({"axis": axis, "sign": sign, "us": us, "counted": m, "crossed": crossed, "segments": segs})
        return m, crossed


def calibrate_tilt(log):
    seq = [(+1, 0), (-1, 0), (+1, 0)]
    for ui in range(1, len(G.CAL_US)):
        seq += [(-1, ui), (+1, ui)]
    seq.append((-1, 0))                                  # repeat: a check on the fit
    print("\n== elevation ==")
    input("  Put the plate at home: facing the horizon, slack taken up for moving UP. Enter")
    last_dir["T"] = +1
    rows = []
    for n, (sign, ui) in enumerate(seq, 1):
        start, target = (0, 90) if sign > 0 else (90, 0)
        m, crossed = measure("T", sign, ui, start, target, n, len(seq), log)
        rows.append((sign, ui, m, crossed))
    # 90 = k[sign, ui] * counted - slack * crossed, least squares over all runs
    cols = sorted({(s, u) for s, u, _, _ in rows})
    A = np.zeros((len(rows), len(cols) + 1))
    y = np.full(len(rows), 90.0)
    for i, (s, u, m, c) in enumerate(rows):
        A[i, cols.index((s, u))] = m
        A[i, -1] = -1.0 if c else 0.0
    if np.linalg.matrix_rank(A) < A.shape[1]:
        sys.exit("Can't separate the slack from the speeds: needs an UP run at the slowest offset from home with the "
                 "slack taken up for moving up (run 1). Rerun.")
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = A @ x - y
    slack = float(x[-1])
    tables = {k: list(G.TABLES[k]) for k in ("TILT_FWD_DPS", "TILT_REV_DPS")}
    print(f"\n  slack {slack:.1f} deg (was {G.TILT_BACKLASH:.1f}); fit residuals "
          + ", ".join(f"{r:+.1f}" for r in resid) + " deg")
    for (s, u), k in zip(cols, x[:-1]):
        kk = key("T", s)
        old = G.TABLES[kk][u]
        tables[kk][u] = old * k
        print(f"  {way('T', s):4s} at {G.CAL_US[u]:.0f} us: {old * k:6.1f} deg/s (was {old:.1f}, {k - 1:+.0%})")
    return tables, slack


def calibrate_pan(log):
    seq = []
    for ui in range(len(G.CAL_US)):
        seq += [(+1, ui), (-1, ui)]
    print("\n== pan ==")
    print("  Start at -90 (plate facing straight out to the side away from mic 4).")
    position("P", -90, hand_ok=False)
    tables = {k: list(G.TABLES[k]) for k in ("PAN_FWD_DPS", "PAN_REV_DPS")}
    for n, (sign, ui) in enumerate(seq, 1):
        start, target = (-90, 90) if sign > 0 else (90, -90)
        m, _ = measure("P", sign, ui, start, target, n, len(seq), log)
        kk = key("P", sign)
        old = G.TABLES[kk][ui]
        tables[kk][ui] = old * 180.0 / m
        print(f"    {way('P', sign)} at {G.CAL_US[ui]:.0f} us: {tables[kk][ui]:.1f} deg/s (was {old:.1f}, {180 / m - 1:+.0%})")
    return tables


def main():
    global st
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=["tilt", "pan", "both"], default="both")
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()
    st = GimbalStream(on_packet=lambda r, g: latest.__setitem__(0, g))
    time.sleep(0.5)
    if latest[0] is None:
        sys.exit("No gimbal packets. Flash it: ~/.platformio/penv/bin/pio run -d tdoa -e gimbal -t upload")
    if latest[0].sim_t > 0:
        print("(gimbal_sim: the servos aren't driven; the sim's own gimbal moves)")
    st.send("E0")
    print(__doc__.split("\n\n")[1] + "\n\n" + __doc__.split("\n\n")[2])
    log, tables, slack = [], {}, None
    try:
        if args.axis in ("tilt", "both"):
            t, slack = calibrate_tilt(log)
            tables.update(t)
        if args.axis in ("pan", "both"):
            tables.update(calibrate_pan(log))
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
        sys.exit(1)
    finally:
        st.send("E0")
        time.sleep(0.1)
        st.close()
        if log:
            os.makedirs("runs", exist_ok=True)
            path = time.strftime("runs/gimbal_cal_%Y%m%d_%H%M%S.json")
            json.dump(log, open(path, "w"), indent=1)
            print(f"Runs saved to {path}")
    for k, v in tables.items():
        if any(b <= a for a, b in zip(v, v[1:])):
            print(f"Note: {k} doesn't increase with the offset; the board skips points that aren't faster")
    print("Each fine-adjust step adds the servo's spin-up, which the board doesn't model: fewer, longer steps are "
          "more accurate.")
    if args.write:
        G.write_tables(tables)
        if slack is not None:
            G.write_define("TILT_BACKLASH_DEG", max(0.0, slack))
        print("\nWritten to src/gimbal_config.h. Put the gimbal at home (slack taken up for moving up), then reflash:\n"
              "  ~/.platformio/penv/bin/pio run -d tdoa -e gimbal -t upload")
    else:
        print("\nNot written (run with --write to save).")


if __name__ == "__main__":
    main()
