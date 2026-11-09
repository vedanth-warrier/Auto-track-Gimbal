"""Guided servo calibration for the FS90MR continuous-rotation servos (tdoa firmware).

  uv run python servotest.py           run all three steps, print the config.h values
  uv run python servotest.py --write   also write them into src/config.h (then reflash)

1. Stop pulses: each servo gets its stop pulse; you adjust it until the servo stands still.
2. Directions: tone source on a stand, 15-40 deg off the array axis. Each axis is driven briefly towards the
   source; if the pointing error grows instead of shrinking, that axis's sign is flipped.
3. Speed scale: each axis is driven at a known commanded speed; the tracker measures how fast the pointing
   actually moved, which corrects TDOA_US_PER_RADS.
The servos are stopped (detached) whenever the script ends, including on Ctrl-C.
"""
import sys
import time
import numpy as np
from tstream import Stream
import tconfig as C

TEST_RATE = 0.4      # rad/s for the direction and speed tests
DRIVE_S = 0.3        # s of driving per test (~7 deg at the test rate)
SETTLE_S = 0.5       # s after stopping before reading the tracker again

latest = [None]
st = Stream(on_result=lambda r: latest.__setitem__(0, r))
a = np.radians(C.TILT_UP_AZ_DEG)
UP = np.array([np.cos(a), np.sin(a)])
RIGHT = np.array([UP[1], -UP[0]])


def error_components():
    """(tilt, pan) pointing error in degrees from the tracked direction, averaged over 0.3 s; None if not tracking."""
    vals = []
    t0 = time.time()
    while time.time() - t0 < 0.3:
        r = latest[0]
        if r is not None and r.track_state == 1:
            e = r.tu[:2]
            vals.append((np.degrees(np.arcsin(np.clip(e @ UP, -1, 1))), np.degrees(np.arcsin(np.clip(e @ RIGHT, -1, 1)))))
        time.sleep(0.02)
    return np.mean(vals, 0) if len(vals) > 5 else None


def drive(axis, rate, secs):
    st.send(f"T{axis}{rate:.3f}")
    time.sleep(secs)
    st.send(f"T{axis}0")


def main():
    out = {}
    time.sleep(1)
    if latest[0] is None:
        raise SystemExit("no packets: flash the tdoa firmware")

    print("\n1. STOP PULSES. Each servo gets a stop pulse; adjust until it stands completely still.")
    print("   Type a pulse in us (e.g. 1495) and Enter to try it; just Enter to accept.")
    for axis, name, key in [("P", "pan", "PAN_NEUTRAL_US"), ("T", "tilt", "TILT_NEUTRAL_US")]:
        us = getattr(C, key)
        st.send("TP0"); st.send("TT0")
        st.send(f"N{axis}{us:.1f}")
        while True:
            s = input(f"   {name}: {us:.1f} us. Still? (Enter = yes, or a new pulse): ").strip()
            if not s:
                break
            us = float(s)
            st.send(f"N{axis}{us:.1f}")
        out[key] = us
    st.send("S")

    print("\n2. DIRECTIONS. Tone on, source on a stand 15-40 deg off the array axis, a bit to one side.")
    input("   Enter when the tracker has it (view.py shows a green dot)...")
    for axis, name, key, idx in [("T", "tilt", "TILT_SIGN", 0), ("P", "pan", "PAN_SIGN", 1)]:
        e0 = error_components()
        if e0 is None:
            print(f"   {name}: not tracking; skipped")
            continue
        if abs(e0[idx]) < 4:
            print(f"   {name}: source only {e0[idx]:+.1f} deg off along this axis; move it further to one side and rerun")
            continue
        drive(axis, TEST_RATE * np.sign(e0[idx]), DRIVE_S)       # + rate moves the pointing towards + error
        time.sleep(SETTLE_S)
        e1 = error_components()
        if e1 is None:
            print(f"   {name}: lost the track after moving; skipped")
            continue
        moved = abs(e0[idx]) - abs(e1[idx])
        if moved > 1:
            print(f"   {name}: error {e0[idx]:+.1f} -> {e1[idx]:+.1f} deg: sign correct")
            out[key] = int(getattr(C, key))
        elif moved < -1:
            print(f"   {name}: error {e0[idx]:+.1f} -> {e1[idx]:+.1f} deg: moved AWAY, sign flipped")
            out[key] = -int(getattr(C, key))
        else:
            print(f"   {name}: error {e0[idx]:+.1f} -> {e1[idx]:+.1f} deg: hardly moved (stop pulse or speed scale off?)")
        # 3. speed scale on the same axis: commanded vs measured, in the direction that reduces the error
        e0 = error_components()
        if e0 is None or abs(e0[idx]) < 4:
            continue
        sign_fix = out.get(key, getattr(C, key)) * getattr(C, key)   # -1 if this run just flipped the sign
        drive(axis, TEST_RATE * np.sign(e0[idx]) * sign_fix, DRIVE_S)
        time.sleep(SETTLE_S)
        e1 = error_components()
        if e1 is None:
            continue
        actual = np.radians(abs(e0[idx] - e1[idx])) / DRIVE_S
        if actual > 0.02:
            k = C.US_PER_RADS * TEST_RATE / actual
            print(f"   {name}: commanded {TEST_RATE:.2f} rad/s, measured {actual:.2f} rad/s -> US_PER_RADS {k:.1f}")
            out.setdefault("_k", []).append(k)
    st.send("S")

    ks = out.pop("_k", [])
    if ks:
        out["US_PER_RADS"] = float(np.mean(ks))
    print("\nFor tdoa/src/config.h:")
    for k, v in out.items():
        print(f"#define TDOA_{k} {v:.1f}f" if isinstance(v, float) else f"#define TDOA_{k} {v}")
    if "--write" in sys.argv:
        C.write_defines(out)
        print("written. Reflash: ~/.platformio/penv/bin/pio run -d tdoa -e tdoa -t upload")


if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            st.send("S")
            time.sleep(0.1)
        finally:
            st.close()
