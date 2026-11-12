"""Closed-loop check of the gimbal controller on the board, env gimbal_sim (no servos are driven).

  uv run python gimbal_simcheck.py [--save FILE.npz]

Restarts the board's 70 s scenario (scenario() in src/gimbal.cpp), switches to automatic and records one run.
Per phase it reports the true pointing error (true plate axis vs true source) in the phase's last 4 s, how much
of that is unavoidable (source outside the travel), time to (re)acquire, servo reversals (oscillation), and
overall whether the true pose stayed inside the travel and how far the dead-reckoned pose drifted from it.
The board's sim gives the servos speed errors (+8 % / -5 % pan, -10 % / +10 % elevation), so drift is expected.
"""
import argparse
import time
import numpy as np
from gstream import GimbalStream
import gconfig as G

PHASES = [  # (start s, end s, label, tone on) as in scenario()
    (0, 10, "still az 40 el 25", True),
    (10, 20, "moving az 40->-50, el 25->45", True),
    (20, 26, "tone off", False),
    (26, 36, "on again at az -70 el 15", True),
    (36, 46, "behind: az 150 el 30", True),
    (46, 56, "near zenith: az -20 el 82", True),
    (56, 66, "near horizon: az 10 el 2", True),
    (66, 70, "tone off", False),
]


def unit(az, el):
    a, e = np.radians(az), np.radians(el)
    return np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], -1)


def ang(u, v):
    return np.degrees(np.arccos(np.clip(np.sum(u * v, -1), -1, 1)))


def best_pose(az, el):
    """Closest reachable pose (inside the travel margin) to each source direction, by brute force."""
    pans = np.arange(G.P_LO, G.P_HI + 0.01, 0.5)
    tilts = np.arange(G.T_LO, G.T_HI + 0.01, 0.5)
    P, T = np.meshgrid(pans, tilts)
    grid = unit(P.ravel(), T.ravel())
    out = []
    for s in unit(az, el):
        out.append(np.max(grid @ s))
    return np.degrees(np.arccos(np.clip(out, -1, 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save")
    args = ap.parse_args()
    rows = []
    st = GimbalStream(on_packet=lambda r, g: rows.append((r, g)))
    time.sleep(0.5)
    if not rows:
        raise SystemExit("No gimbal packets. Flash it: pio run -d tdoa -e gimbal_sim -t upload")
    if rows[-1][1].sim_t == 0 and rows[-2][1].sim_t == 0:
        raise SystemExit("This is the real gimbal firmware, not gimbal_sim")
    st.send("R")
    st.send("E1")
    time.sleep(0.2)
    rows.clear()
    print("Recording one 70 s run...")
    t0 = time.time()
    while time.time() - t0 < 75:
        if len(rows) > 100 and rows[-1][1].sim_t < rows[-100][1].sim_t:   # scenario wrapped
            break
        time.sleep(0.2)
    st.send("E0")
    data = list(rows)
    st.close()
    g = [x[1] for x in data]
    r = [x[0] for x in data]
    t = np.array([x.sim_t for x in g])
    keep = np.r_[True, np.diff(t) > 0]          # stop at the wrap
    cut = np.argmin(keep) if not keep.all() else len(t)
    g, r, t = g[:cut], r[:cut], t[:cut]
    n = len(t)
    print(f"{n} packets over {t[-1] - t[0]:.1f} s ({n / max(t[-1] - t[0], 1e-9):.1f}/s)")

    def col(name):
        return np.array([getattr(x, name) for x in g], float)

    pan, tilt, span, stilt = col("pan"), col("tilt"), col("sim_pan"), col("sim_tilt")
    saz, sel, tone = col("sim_src_az"), col("sim_src_el"), col("sim_tone")
    state, prate, trate = col("state"), col("pan_rate"), col("tilt_rate")
    axis, src = unit(span, stilt), unit(saz, sel)
    err = ang(axis, src)
    floor = best_pose(saz, sel)
    trk = np.array([x.track_state for x in r])
    if args.save:
        np.savez(args.save, t=t, pan=pan, tilt=tilt, sim_pan=span, sim_tilt=stilt, src_az=saz, src_el=sel,
                 tone=tone, state=state, pan_rate=prate, tilt_rate=trate, err=err, floor=floor, track=trk)

    names = ["idle", "tracking", "holding", "searching", "parked"]
    print(f"\n{'phase':34s} {'acquire':>8s} {'err med/p95/max':>17s} {'unavoid':>8s} {'rev P/T':>8s}  states")
    for a, b, label, on in PHASES:
        m = (t >= a) & (t < b)
        if not m.any():
            continue
        seen = np.flatnonzero(m & (state == 1))
        acq = f"{t[seen[0]] - a:6.2f} s" if on and len(seen) else ("   never" if on else "       -")
        s = m & (t >= b - 4)
        if on:
            e = err[s]
            errs = f"{np.median(e):5.1f}/{np.percentile(e, 95):5.1f}/{e.max():5.1f}"
            unav = f"{np.median(floor[s]):6.1f}"
        else:
            errs, unav = "                -", "       -"
        rev = []
        for rate in (prate, trate):
            sg = np.sign(rate[s])
            sg = sg[sg != 0]
            rev.append(int(np.sum(sg[1:] != sg[:-1])))
        frac = {names[k]: np.mean(state[m] == k) for k in range(5) if np.mean(state[m] == k) > 0.005}
        fs = " ".join(f"{k} {v:.0%}" for k, v in frac.items())
        print(f"{label:34s} {acq:>8s} {errs:>17s} {unav:>8s} {rev[0]:>3d}/{rev[1]:<3d}  {fs}")

    print("\nerr = true angle between plate axis and source in the phase's last 4 s (deg); unavoid = median of the")
    print("best reachable error there (source outside the travel); rev = servo direction reversals in that window.")
    print(f"\nTrue pose range: pan {span.min():.1f} .. {span.max():.1f} (allowed {G.PAN_MIN:.0f} .. {G.PAN_MAX:.0f}), "
          f"elevation {stilt.min():.1f} .. {stilt.max():.1f} (allowed {G.TILT_MIN:.0f} .. {G.TILT_MAX:.0f})")
    print(f"Dead-reckoning drift at the end (estimate - true): pan {pan[-1] - span[-1]:+.1f}, "
          f"elevation {tilt[-1] - stilt[-1]:+.1f} deg; largest |pan| {np.max(np.abs(pan - span)):.1f}, "
          f"|elevation| {np.max(np.abs(tilt - stilt)):.1f}")
    print(f"Packets dropped by the board (nobody reading): {g[-1].tx_drops}")


if __name__ == "__main__":
    main()
