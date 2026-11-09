"""What the mic geometry in src/config.h can resolve (no board needed).

Phase differences repeat every wavelength, so pairs further apart than half a wavelength can also fit "ghost"
directions. For a true direction, a ghost is any direction outside the true direction's own peak (the
connected region where the fit stays >= GHOST_FIT) whose predicted pair phases fit at least GHOST_FIT
(1 = identical phases). Uses the same steered response grid as the board.

  uv run python ambiguity.py           report for TDOA_TONE_HZ in config.h, and the cone to use
  uv run python ambiguity.py --sweep   compare tone frequencies: worst ghost fit and angular precision
"""
import sys
import numpy as np
from tconfig import TONE_HZ, SPEED, PAIR_NAMES, PAIR_DIST, CONE_DEG, RATE
import tdoa_ref as R

GHOST_FIT = 0.8
PHASE_NOISE = 0.05    # rad rms per pair, for the precision estimate (about a 20 dB SNR tone)
BIN = RATE / R.NFFT


def neighbours():
    # grid adjacency (same ordering as tdoa_ref.grid: rings of N_AZ by elevation, then the zenith)
    n_az, n_el = int(360 / R.AZ_STEP + 0.5), int(90 / R.EL_STEP + 0.5)
    nb = [[] for _ in range(len(R.GU))]
    for e in range(n_el):
        for a in range(n_az):
            g = e * n_az + a
            nb[g] += [e * n_az + (a + 1) % n_az, e * n_az + (a - 1) % n_az]
            nb[g].append((e + 1) * n_az + a if e + 1 < n_el else n_el * n_az)
            if e > 0:
                nb[g].append((e - 1) * n_az + a)
    nb[n_el * n_az] = [(n_el - 1) * n_az + a for a in range(n_az)]
    return nb


NB = neighbours()


def worst_ghost(f, el_min=0):
    """Worst (highest) ghost fit over true directions with elevation >= el_min, and the nearest ghost angle."""
    theta = 2 * np.pi * f * (R.GU @ R.PD.T) / SPEED
    steer = np.exp(1j * theta)
    worst, nearest = 0.0, np.inf
    for g0 in np.flatnonzero(np.degrees(np.arcsin(R.GU[:, 2])) >= el_min):
        fit = (steer[g0][None, :] * np.conj(steer)).real.sum(1) / 6
        lobe = {g0}; todo = [g0]                     # flood-fill the true direction's own peak
        while todo:
            for n in NB[todo.pop()]:
                if n not in lobe and fit[n] >= GHOST_FIT:
                    lobe.add(n); todo.append(n)
        out = np.ones(len(fit), bool); out[list(lobe)] = False
        peak = np.array([all(fit[g] >= fit[m] for m in NB[g]) for g in range(len(fit))])   # local maxima
        out &= peak
        if out.any():
            w = fit[out].max()
            worst = max(worst, w)
            if w >= GHOST_FIT:
                ang = np.degrees(np.arccos(np.clip(R.GU[out] @ R.GU[g0], -1, 1)))
                nearest = min(nearest, ang[fit[out] >= GHOST_FIT].min())
    return worst, nearest


def precision(f):
    """Angular error (deg rms) near the axis for PHASE_NOISE per pair, from the least-squares solve."""
    A = R.PD[:, :2]
    cov = np.linalg.inv(A.T @ A) * (PHASE_NOISE * SPEED / (2 * np.pi * f)) ** 2
    return np.degrees(np.sqrt(np.trace(cov)))


lam = SPEED / TONE_HZ
print("pair distances (mm): " + ", ".join(f"{n} {1000 * d:.1f}" for n, d in zip(PAIR_NAMES, PAIR_DIST)))
if "--sweep" in sys.argv:
    print(f"\n{'tone Hz':>8} {'on bin':>7} {'worst other peak':>16} {'nearest ghost':>14} {'precision':>10}  (whole hemisphere, el >= 0)")
    for f in (np.arange(6000, 10001, 250) if "--fine" in sys.argv else np.arange(2000, 15001, 500)):
        fb = round(f / BIN) * BIN
        w, d = worst_ghost(fb)
        print(f"{fb:8.0f} {'yes':>7} {w:16.2f} {('none' if np.isinf(d) else f'{d:.0f} deg'):>14} {precision(fb):9.2f}d")
    print(f"\nprecision = rms angle error near the axis for {PHASE_NOISE} rad of phase noise per pair; it scales with 1/f.")
    print(f"ghost fit < {GHOST_FIT} everywhere = the board can search the whole sky (TDOA_CONE_DEG 90).")
    sys.exit()

print(f"tone {TONE_HZ:.0f} Hz, wavelength {1000 * lam:.1f} mm, half wavelength {500 * lam:.1f} mm")
w0, d0 = worst_ghost(TONE_HZ)
w45, d45 = worst_ghost(TONE_HZ, el_min=45)
print(f"whole hemisphere: worst ghost fit {w0:.2f}, nearest ghost {'none' if np.isinf(d0) else f'{d0:.0f} deg'}")
print(f"within 45 deg of the axis: worst ghost fit {w45:.2f}, nearest ghost {'none' if np.isinf(d45) else f'{d45:.0f} deg'}")
print(f"precision near the axis: {precision(TONE_HZ):.2f} deg rms per {PHASE_NOISE} rad of pair phase noise")
if np.isinf(d0):
    print("\nNo ghosts anywhere: set TDOA_CONE_DEG to 90 (search the whole sky)." + ("" if CONE_DEG >= 90 else f" Currently {CONE_DEG:g}."))
else:
    suggest = int(min(d0, d45) / 2) // 5 * 5
    print(f"\nSuggested TDOA_CONE_DEG: {suggest} (currently {CONE_DEG:g}); the source must start inside it.")
