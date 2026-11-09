"""Circle test: calibrates bias and geometry scale from real directions.

Setup: tone source on a stand at a known angle off the array axis (e.g. 40 deg), pointing at the array.
Rotate the ARRAY slowly about its own axis through at least one full turn (about 20-40 s) while this records;
don't move the source. Seen in the array frame, the source then goes round a circle of radius sin(off-axis)
in (ux, uy). The fit gives:
  * centre: a constant bias in every direction (from pair-delay errors) -> corrected MIC_DELAY_US
  * radius vs the known angle: geometry scale (MIC_POS or speed of sound) error
  * how far the points stray from a circle: direction-dependent error (reflections, mic phase patterns)
    that a delay correction can't remove

  uv run python circle.py --off 40            record 40 s, fit, print the corrected MIC_DELAY_US
  uv run python circle.py --off 40 --write    also write it into src/config.h (then reflash)
  uv run python circle.py --off 40 --steps 12 step mode: rotate ~30 deg, hands away, Enter; 2 s per step
                                              (keeps your hands out of the sound field while recording)
  uv run python circle.py --file runs/X.npz   re-analyse a saved run
  uv run python circle.py --compare A.npz B.npz   is the wobble pattern the same in both runs?

Every run is saved in host/runs/. The wobble is broken into how many times per turn it repeats:
  2 per turn = ellipse (a MIC_POS error along one axis); 4 per turn = tied to the 4 mics / shrouds
  (repeatable, so a correction table could remove it); irregular or different between runs = reflections.
"""
import os
import sys
import time
import numpy as np
from tconfig import MIC_POS, SPEED, MIC_DELAY_US

REC_S = 40


def arg(name, default=None):
    return float(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else default


def fit_circle(x, y):
    """Least-squares circle: centre (cx, cy), radius."""
    A = np.column_stack([x, y, np.ones_like(x)])
    b = -(x ** 2 + y ** 2)
    D, E, F = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = -D / 2, -E / 2
    return cx, cy, np.sqrt(max(cx * cx + cy * cy - F, 0))


def radial_pattern(ux, uy, cx, cy, r):
    """Radial error (deg) vs angle around the circle centre, and its harmonics (deg amplitude, k = 1..6)."""
    th = np.arctan2(uy - cy, ux - cx)
    dev = np.degrees(np.arcsin(np.clip(np.hypot(ux - cx, uy - cy), 0, 1)) - np.arcsin(min(r, 1)))
    cols = [np.ones_like(th)] + [f(k * th) for k in range(1, 7) for f in (np.cos, np.sin)]
    c, *_ = np.linalg.lstsq(np.column_stack(cols), dev, rcond=None)
    harm = np.hypot(c[1::2], c[2::2])
    return th, dev, harm


def binned(th, dev, nbin=36):
    """Mean radial error in angle bins (for comparing runs); NaN where a bin is empty."""
    b = ((np.degrees(th) % 360) // (360 / nbin)).astype(int)
    return np.array([dev[b == i].mean() if np.any(b == i) else np.nan for i in range(nbin)])


def bias_to_delays(cx, cy):
    """Per-mic delay change (us) that shifts every estimate by -(cx, cy)."""
    d = -1e6 * (MIC_POS[:, 0] * cx + MIC_POS[:, 1] * cy) / SPEED
    return d - d.mean()


def analyse(ux, uy, off_deg=None):
    cx, cy, r = fit_circle(ux, uy)
    ang = np.degrees(np.arctan2(uy - cy, ux - cx))
    cover = 360 - np.degrees(np.max(np.diff(np.sort(np.radians(ang)) % (2 * np.pi), append=2 * np.pi + np.radians(np.sort(ang)[0]) - np.radians(np.sort(ang)[-1]))))
    rr = np.hypot(ux - cx, uy - cy)
    out = dict(cx=cx, cy=cy, r=r, cover=cover,
               bias_deg=np.degrees(np.arcsin(min(1, np.hypot(cx, cy)))),
               wobble_deg=np.degrees(np.std(np.arcsin(np.clip(rr, -1, 1)))),
               mag_range=(np.degrees(np.arcsin(np.clip(np.hypot(ux, uy), 0, 1))).min(),
                          np.degrees(np.arcsin(np.clip(np.hypot(ux, uy), 0, 1))).max()),
               delays=bias_to_delays(cx, cy))
    if off_deg is not None:
        out["scale"] = r / np.sin(np.radians(off_deg))    # > 1: MIC_POS too small (or speed of sound too high)
    out["th"], out["dev"], out["harm"] = radial_pattern(ux, uy, cx, cy, r)
    return out


def report(a, off):
    print(f"circle covers {a['cover']:.0f} deg of rotation" + ("" if a["cover"] > 300 else "  (WARNING: rotate a full turn)"))
    print(f"off-axis angle seen: {a['mag_range'][0]:.1f} to {a['mag_range'][1]:.1f} deg (true {off:g})  <- the 'size' swinging with direction")
    print(f"circle centre (bias): ux {a['cx']:+.3f}, uy {a['cy']:+.3f}  = {a['bias_deg']:.1f} deg of constant offset")
    print(f"circle radius {a['r']:.3f} -> {np.degrees(np.arcsin(min(1, a['r']))):.1f} deg off axis; scale vs true: {a['scale']:.3f}"
          + ("   (MIC_POS spacing or speed of sound off by this factor, or the source angle isn't exactly as given)"
             if abs(a['scale'] - 1) > 0.05 else "   (geometry scale fine)"))
    print(f"wobble around the circle: {a['wobble_deg']:.1f} deg rms (direction-dependent error a delay correction can't remove)")
    h = a["harm"]
    print("wobble by repeats per turn (deg amplitude): " + "  ".join(f"{k}x {v:.1f}" for k, v in enumerate(h, 1)))
    k = int(np.argmax(h)) + 1
    note = {2: "ellipse: a MIC_POS error along one axis", 4: "tied to the 4 mics / shrouds (check repeatability with --compare)",
            1: "rotation not about the array centre, or the source moved", 3: "odd pattern: likely reflections", 5: "likely reflections",
            6: "likely reflections"}[k]
    print(f"largest: {k} per turn -> {note}")


def save(ux, uy, off):
    d = os.path.join(os.path.dirname(__file__), "runs")
    os.makedirs(d, exist_ok=True)
    gi = os.path.join(d, ".gitignore")
    if not os.path.exists(gi):
        open(gi, "w").write("*\n")
    path = os.path.join(d, time.strftime("circle_%Y%m%d_%H%M%S.npz"))
    np.savez(path, ux=ux, uy=uy, off=off)
    print(f"saved {os.path.relpath(path, os.path.dirname(__file__))}")


def compare(fa, fb, nbin=36):
    A, B = np.load(fa), np.load(fb)
    pa = binned(*radial_pattern(A["ux"], A["uy"], *fit_circle(A["ux"], A["uy"]))[:2], nbin)
    pb = binned(*radial_pattern(B["ux"], B["uy"], *fit_circle(B["ux"], B["uy"]))[:2], nbin)
    ok = np.isfinite(pa) & np.isfinite(pb)
    c = np.corrcoef(pa[ok], pb[ok])[0, 1]
    print(f"wobble pattern vs angle, {ok.sum()} of {nbin} bins in both runs:")
    print(f"  correlation {c:+.2f}, rms difference {np.sqrt(np.mean((pa[ok] - pb[ok]) ** 2)):.1f} deg, rms size {np.sqrt(np.nanmean(pa ** 2)):.1f} / {np.sqrt(np.nanmean(pb ** 2)):.1f} deg")
    print("  correlation near +1: the same pattern both times (array-fixed, repeatable: a correction table would work)")
    print("  near 0: different patterns (reflections / setup): calibration can't fix it; change the environment")


def main():
    if "--compare" in sys.argv:
        i = sys.argv.index("--compare")
        compare(sys.argv[i + 1], sys.argv[i + 2])
        return
    if "--file" in sys.argv:
        f = np.load(sys.argv[sys.argv.index("--file") + 1])
        off = float(f["off"])
        a = analyse(f["ux"], f["uy"], off)
        report(a, off)
        return
    from tstream import Stream
    off = arg("--off")
    if off is None:
        raise SystemExit("give the source's angle off the array axis: --off DEG")
    steps = arg("--steps")
    rs = []
    rec = [True]
    st = Stream(on_result=lambda r: rs.append(r) if rec[0] else None)
    if steps:
        n = int(steps)
        ux, uy = [], []
        for k in range(n):
            input(f"step {k + 1}/{n}: rotate the array ~{360 / n:.0f} deg (from the start, {k * 360 / n:.0f} deg), hands away, Enter")
            time.sleep(0.5)
            rs.clear()
            time.sleep(2)
            v = [r for r in rs if r.valid]
            print(f"  {len(v)} estimates")
            ux += [r.u[0] for r in v]; uy += [r.u[1] for r in v]
        st.close()
        ux, uy = np.array(ux), np.array(uy)
    else:
        input(f"Source fixed at {off:g} deg off axis, tone on. Enter, then rotate the array slowly through a full turn ({REC_S} s).")
        time.sleep(0.5)
        rs.clear()
        for s_ in range(REC_S, 0, -10):
            print(f"  {s_} s left...")
            time.sleep(10)
        st.close()
        v = [r for r in rs if r.valid]
        ux = np.array([r.u[0] for r in v]); uy = np.array([r.u[1] for r in v])
    if len(ux) < 100:
        raise SystemExit(f"only {len(ux)} estimates with the tone: check it's playing at the tracker's frequency, loud")
    save(ux, uy, off)
    a = analyse(ux, uy, off)
    print(f"\n{len(ux)} estimates")
    report(a, off)
    new = MIC_DELAY_US + a["delays"]
    print("\nDelay change per mic (us): " + "  ".join(f"mic{m + 1} {d:+6.2f}" for m, d in enumerate(a["delays"])))
    print("Corrected, for tdoa/src/config.h:")
    print("static const float MIC_DELAY_US[4] = {" + ", ".join(f"{d:.3f}f" for d in new) + "};")
    if "--write" in sys.argv:
        from calibrate import write_config
        write_config(new)


if __name__ == "__main__":
    main()
