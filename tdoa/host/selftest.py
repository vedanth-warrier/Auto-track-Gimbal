"""Offline self-test of the TDOA algorithm (Python reference, no board): synthetic plane-wave tones.

1. Still sources inside the search cone, with noise: direction error.
2. A source moving like the board's sim (az 20 deg/s, el 90 -> 45 -> 90 over 20 s): does tracking follow it
   out of the axis cone?
3. Sources that start outside the cone: shows what happens (a planar array's ghosts are exact, so the
   tracker can lock onto one; acquisition needs the source inside the cone).
"""
import numpy as np
from tconfig import CONE_DEG, RATE
import tdoa_ref as R

rng = np.random.default_rng(0)
ang = lambda a, b: np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1)))


def run(dirs, noise, n_pkt):
    # dirs(k) -> unit vector for packet k; returns list of (truth, result)
    T = R.Tracker(); out = []
    for k in range(n_pkt):
        u = dirs(k)
        blk = R.plane_wave(u, R.HOP, amp=0.01, noise=noise, rng=rng, t0=k * R.HOP)
        r = T.step(blk)
        out.append((u, r))
    return out


print(f"cone {CONE_DEG:g} deg")
print("1. still sources (2 s each, last 1 s scored), el 90 down to the cone or 10 deg")
for noise, label in [(1e-4, "tone 34 dB above white noise per sample"), (1e-3, "14 dB"), (3e-3, "4 dB")]:
    errs = []; miss = 0; flags = []
    for el in ([90, 75, 60, 45, 30, 20, 10] if CONE_DEG >= 90 else [90, 85, 80, 75, 72]):
        for az in range(0, 360, 45):
            res = run(lambda k: R.unit(az, el), noise, 250)
            good = [ang(u, r["u"]) for u, r in res[125:] if r is not None]
            miss += 125 - len(good)
            errs += good
            flags.extend(r["ambiguity"] for _, r in res[125:] if r is not None)
    errs = np.array(errs)
    print(f"   {label}: median error {np.median(errs):.2f} deg, 99th pct {np.percentile(errs, 99):.2f}, max {errs.max():.2f}, packets without output {miss}")

print("2. moving source (el 90 -> 45 -> 90, az 20 deg/s), 40 s")
path = lambda k: R.unit((20 * k * R.HOP / RATE) % 360, 67.5 + 22.5 * np.cos(2 * np.pi * (k * R.HOP / RATE) / 20))
res = run(path, 1e-4, int(40 * RATE / R.HOP))
e = np.array([ang(u, r["u"]) for u, r in res if r is not None])
fl = np.mean([r["edge"] for _, r in res if r is not None])
print(f"   median error {np.median(e):.2f} deg, 99th pct {np.percentile(e, 99):.2f}, max {e.max():.2f}, "
      f"packets > 5 deg off {np.sum(e > 5)} of {len(e)}, flagged {100 * fl:.1f}%")

print("3. sources starting outside the cone (tracker starts at the axis), error after 2 s")
for el in [65, 60, 50, 40, 30]:
    e = []
    for az in range(0, 360, 45):
        res = run(lambda k: R.unit(az, el), 1e-4, 250)
        e.append(ang(res[-1][0], res[-1][1]["u"]) if res[-1][1] is not None else np.nan)
    e = np.array(e)
    print(f"   el {el} ({90 - el} deg off axis): found correctly (< 2 deg) at {np.sum(e < 2)} of 8 azimuths; errors {np.round(e).astype(int).tolist()}")
