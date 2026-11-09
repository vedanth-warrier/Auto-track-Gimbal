"""Checks the board's tracker. Needs tdoa_simbad (synthetic source + a fake reflection null on mic 2 for 0.3 s
every 5 s) or tdoa_sim.

1. Board tracker vs the Python tracker fed with the board's own raw estimates (same maths, so should agree).
2. Error vs the synthetic truth: raw estimates vs tracked direction vs servo command, in clean stretches and
   during the fake nulls.
"""
import time
import numpy as np
from tstream import Stream
import tdoa_ref as R

REC_S = 30
import glob, serial
_s = serial.Serial(sorted(glob.glob("/dev/cu.usbmodem*"))[0]); _s.dtr = False; _s.rts = True; time.sleep(0.2); _s.rts = False; _s.close()
time.sleep(1.0)   # board reset: its tracker starts from scratch, like the Python one below
rs = []
st = Stream(on_result=lambda r: rs.append(r))
time.sleep(REC_S)
st.close()
if not any(r.sim_el for r in rs):
    raise SystemExit("no sim truth: flash tdoa_simbad (or tdoa_sim)")
ang = lambda a, b: np.degrees(np.arccos(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-30), -1, 1)))

# Python tracker from scratch, fed the board's raw estimates (both trackers start empty after the reset)
T = R.DirTracker()
d_t, d_c = [], []
for r in rs:
    state, t, c = T.step(r.u if r.valid else None, r.quality)
    if t is not None and r.track_state:
        d_t.append(ang(t, r.tu))
    if c is not None and np.any(r.cmd):
        d_c.append(ang(c, r.cmd))
print(f"{len(rs)} packets ({len(rs) / 125:.0f} s)")
print(f"board tracker vs Python tracker: tracked max {max(d_t):.3f} deg, servo command max {max(d_c):.3f} deg")

# vs truth: the sim's time isn't in the packet, so classify by quality (the fake null drops it) with a margin
truth = [R.unit(r.sim_az, r.sim_el) for r in rs]
bad = np.array([r.valid and r.quality < 0.9 for r in rs])
near_bad = np.convolve(bad, np.ones(60), "same") > 0        # +/- 0.25 s around the corrupted stretches
raw = np.array([ang(r.u, u) if r.valid else np.nan for r, u in zip(rs, truth)])
trk = np.array([ang(r.tu, u) if r.track_state else np.nan for r, u in zip(rs, truth)])
cmd = np.array([ang(r.cmd, u) for r, u in zip(rs, truth)])
for name, m in [("clean stretches", ~near_bad), ("around the fake nulls", near_bad)]:
    if m.any():
        print(f"{name:22s}: raw median {np.nanmedian(raw[m]):5.2f} max {np.nanmax(raw[m]):5.1f} | tracked median {np.nanmedian(trk[m]):5.2f} "
              f"max {np.nanmax(trk[m]):5.1f} | command median {np.nanmedian(cmd[m]):5.2f} max {np.nanmax(cmd[m]):5.1f} deg")
states = np.array([r.track_state for r in rs])
print(f"track state: tracking {100 * np.mean(states == 1):.1f}%, coasting {100 * np.mean(states == 2):.1f}%, lost {100 * np.mean(states == 0):.1f}%")
steps = np.array([ang(a.cmd, b.cmd) for a, b in zip(rs[1:], rs)])
print(f"servo command: moved in {100 * np.mean(steps > 1e-3):.0f}% of packets, largest single step {steps.max():.2f} deg")
