"""Offline test of the direction tracker (tdoa_ref.DirTracker) on a synthetic scenario:
moving source at 1 rad/s with 0.6 deg jitter, a 0.3 s bad-quality excursion (ignored: quality < TRK_Q_MIN),
an 80 ms good-quality excursion (averaged in, and speed-limited), a real sudden move at 8 s, then a still source."""
import numpy as np
import tdoa_ref as R

rng = np.random.default_rng(0); dt = R.HOP / R.RATE
ang = lambda a, b: np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1)))
T = R.DirTracker(); n = int(12 / dt)
truth, meas, out = [], [], []
for k in range(n):
    t = k * dt
    az = np.degrees(1.0 * t / np.sin(np.radians(40))) if t < 8 else 200.0
    u = R.unit(az, 50); truth.append(u)
    m = u + np.radians(0.6) * rng.standard_normal(3); m /= np.linalg.norm(m); q = 0.99
    if 3.0 <= t < 3.3: m = R.unit(az + 140, 35); q = 0.4
    if 5.0 <= t < 5.08: m = R.unit(az + 90, 40); q = 0.95
    meas.append(m); out.append(T.step(m, q))
raw = np.array([ang(m, u) for m, u in zip(meas, truth)])
trk = np.array([ang(o[1], u) if o[1] is not None else np.nan for o, u in zip(out, truth)])
cmd = np.array([ang(o[2], u) for o, u in zip(out, truth)])
seg = lambda a, lo, hi: a[int(lo / dt):int(hi / dt)]
print(f"moving at 1 rad/s: raw error median {np.median(seg(raw, .5, 2.9)):.2f}, tracked {np.median(seg(trk, .5, 2.9)):.2f}, "
      f"servo command {np.median(seg(cmd, .5, 2.9)):.2f} deg (deadband {R.DEADBAND_DEG})")
print(f"0.3 s bad-quality excursion: raw max {seg(raw, 3, 3.3).max():.0f} deg, tracked max {seg(trk, 3, 3.4).max():.2f} deg")
print(f"80 ms good-quality excursion: raw max {seg(raw, 5, 5.08).max():.0f} deg, tracked max {seg(trk, 5, 5.2).max():.2f} deg")
re = next(k for k in range(int(8 / dt), n) if trk[k] < 3) * dt - 8
print(f"real sudden move at 8 s (~107 deg): tracked within 3 deg after {1000 * re:.0f} ms (speed limit {R.TRK_MAX_RATE} rad/s)")
st_ = np.array([o[1] for o in out if o[1] is not None]); stp = [ang(a, b) for a, b in zip(st_[1:], st_)]
print(f"largest step of the tracked direction in one packet: {max(stp):.2f} deg (limit {np.degrees(R.TRK_MAX_RATE * dt):.2f})")
still = [ang(o[1], out[int(10 / dt)][1]) for o in out[int(9.5 / dt):]]
cmd_still = [ang(o[2], out[int(10 / dt)][2]) for o in out[int(9.5 / dt):]]
print(f"still source (0.6 deg raw jitter): tracked moves {np.std(still):.2f} deg rms; servo command moves {max(cmd_still):.2f} deg max")
