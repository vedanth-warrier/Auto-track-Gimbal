"""Records the tracker while it jitters, then works out what causes each sudden jump.

  uv run python jumps.py                 record 30 s (tone on; do whatever makes it jump), save, analyse
  uv run python jumps.py --secs 60       longer recording
  uv run python jumps.py --file runs/X   re-analyse a saved recording

The audio is replayed through the Python reference (same results as the board) to see, at each jump:
fit quality, SNR, each mic's level at the tone (a mic in a reflection null reads well below the others),
whether the new direction was the steered response's runner-up (a near-ghost), and how soon after the tone
was (re)acquired. Excursion = the direction straying more than EXC_DEG from its own running median
(+/- 0.5 s); its causes are judged over the whole excursion plus the 10 packets before it (the averaging
spreads a cause over several packets).
"""
import os
import sys
import time
import numpy as np
import tdoa_ref as R
from tconfig import SPEED

EXC_DEG = 8            # deg from the running median direction = an excursion
NULL_DB = 6            # a mic this far below the median of the others = in a null
ang = lambda a, b: np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1)))


def record(secs):
    from tstream import Stream
    blocks, res = [], []
    st = Stream(on_audio=lambda b: blocks.append(b.astype(np.float32)), on_result=lambda r: res.append(r))
    time.sleep(1)
    blocks.clear(); res.clear()
    print(f"recording {secs} s...")
    time.sleep(secs)
    st.close()
    n = min(len(blocks), len(res))
    board = np.array([[*r.u, r.az, r.el, r.quality, r.ambiguity, r.edge, r.freq, r.snr] for r in res[:n]])
    d = os.path.join(os.path.dirname(__file__), "runs")
    os.makedirs(d, exist_ok=True)
    if not os.path.exists(os.path.join(d, ".gitignore")):
        open(os.path.join(d, ".gitignore"), "w").write("*\n")
    path = os.path.join(d, time.strftime("jumps_%Y%m%d_%H%M%S.npz"))
    np.savez_compressed(path, audio=np.hstack(blocks[:n]), board=board)
    print(f"saved {os.path.relpath(path, os.path.dirname(__file__))}")
    return path


def replay(audio):
    """Reference tracker on the recording; per packet: result dict (or None) plus diagnostics."""
    T = R.Tracker(); out = []
    for i in range(audio.shape[1] // R.HOP):
        r = T.step(audio[:, i * R.HOP : (i + 1) * R.HOP].astype(float))
        if r is None:
            out.append(None)
            continue
        k = T.k
        lv = 10 * np.log10((np.abs(T.X[:, k - 1 : k + 2]) ** 2).sum(1) + 1e-30)   # per-mic power at the tone
        coh = np.abs(T.G)                                   # 1 = pair phase steady over the averaging time
        P = (T.G[None, :] * np.conj(R.STEER)).real.sum(1)
        out.append(dict(r, levels=lv, coh=coh, run=T.run, P=P, single=R.solve(T.g)["u"]))
    return out


def analyse(path):
    f = np.load(path)
    audio, board = f["audio"], f["board"]
    out = replay(audio)
    n = len(out)
    valid = [i for i in range(n) if out[i] is not None]
    print(f"\n{n} packets ({n / 125:.1f} s), tone tracked in {len(valid)} ({100 * len(valid) / max(n, 1):.0f}%)")
    bv = [i for i in valid if i < len(board) and board[i, 7] > 0]
    if bv:
        d = [ang(out[i]["u"], board[i, :3]) for i in bv]
        print(f"board vs Python reference on this recording: median {np.median(d):.2f} deg, max {np.max(d):.2f} deg")

    U = np.array([out[i]["u"] for i in valid])
    # reference direction: running median over +/- 0.5 s (robust to excursions shorter than that)
    ref = np.empty_like(U)
    for k in range(len(U)):
        m = np.median(U[max(0, k - 62) : k + 63], axis=0)
        ref[k] = m / np.linalg.norm(m)
    dev = np.array([ang(U[k], ref[k]) for k in range(len(U))])
    flips = sum(1 for k in range(1, len(U)) if abs((out[valid[k]]["az"] - out[valid[k - 1]]["az"] + 180) % 360 - 180) > 90
                and ang(U[k], U[k - 1]) < 5)
    print(f"azimuth flips near the axis (big azimuth change, tiny real angle; harmless): {flips}")
    # excursions: runs of packets more than EXC_DEG from the running median
    events, k = [], 0
    while k < len(U):
        if dev[k] > EXC_DEG:
            e = k
            while e + 1 < len(U) and dev[e + 1] > EXC_DEG and valid[e + 1] - valid[e] <= 3:
                e += 1
            events.append((k, e)); k = e + 1
        else:
            k += 1
    calm = dev[dev <= EXC_DEG]
    print(f"steady jitter (packets within {EXC_DEG} deg of the running median): median {np.median(calm):.2f} deg, "
          f"95th pct {np.percentile(calm, 95):.2f} deg")
    q_all = np.array([out[i]["quality"] for i in valid])
    print(f"fit quality overall: median {np.median(q_all):.3f}, 5th pct {np.percentile(q_all, 5):.3f}")
    # what the tracker makes of it (same tracker as the board)
    T = R.DirTracker(); tu = []
    for o_ in out:
        st_, t_, c_ = T.step(o_["u"] if o_ is not None else None, o_["quality"] if o_ is not None else 0)
        tu.append(t_)
    tdev = np.array([ang(tu[i], ref[k]) for k, i in enumerate(valid) if tu[i] is not None])
    print(f"tracker output: {np.sum(tdev > EXC_DEG)} packets more than {EXC_DEG} deg from the running median "
          f"(raw: {np.sum(dev > EXC_DEG)}); tracked jitter median {np.median(tdev):.2f} deg, max {tdev.max():.1f} deg")
    print(f"excursions (> {EXC_DEG} deg from the running median): {len(events)}, "
          f"{100 * sum(e - s_ + 1 for s_, e in events) / len(U):.1f}% of packets")
    if not events:
        return

    causes = dict(acq=0, lowq=0, null=0, snr=0, ghost=0)
    rows = []
    for s_, e in events:
        idx = [valid[k] for k in range(max(0, s_ - 10), e + 1)]          # include the run-up (averaging)
        peak = valid[s_ + int(np.argmax(dev[s_ : e + 1]))]
        rel = np.array([[out[i]["levels"][m] - np.median(np.delete(out[i]["levels"], m)) for m in range(4)] for i in idx])
        worst = rel.min(0)
        nulls = [m + 1 for m in range(4) if worst[m] < -NULL_DB]
        qmin = min(out[i]["quality"] for i in idx[10:] or idx)
        smin = min(out[i]["snr"] for i in idx)
        far = R.GU @ ref[valid.index(peak)] < np.cos(np.radians(30))
        runner = R.GU[far][np.argmax(out[peak]["P"][far])] if far.any() else None
        c = dict(acq=out[valid[s_]]["run"] < 20, lowq=qmin < np.percentile(q_all, 5), null=bool(nulls), snr=smin < 30,
                 ghost=runner is not None and ang(runner, out[peak]["u"]) < 10)
        for key, v in c.items():
            causes[key] += v
        rows.append((valid[s_], (valid[e] - valid[s_] + 1) * 8, dev[s_ : e + 1].max(), qmin, smin, worst, nulls, out[valid[s_]]["run"]))
    n_ev = len(events)
    print("\nwhat coincides with the excursions (one can have several):")
    print(f"  started within 20 packets of (re)acquiring the tone: {causes['acq']} of {n_ev}")
    print(f"  fit quality dropped into the worst 5%:               {causes['lowq']} of {n_ev}")
    print(f"  a mic fell {NULL_DB} dB below the others (null):          {causes['null']} of {n_ev}")
    print(f"  SNR dropped under 30 dB:                             {causes['snr']} of {n_ev}")
    print(f"  went to the runner-up direction (near-ghost):        {causes['ghost']} of {n_ev}")
    print("\nexcursions: packet  ms   max dev  min quality  min SNR  worst level vs others mic1..4 (dB)  null  run")
    for b, ms, mx, q, sn, worst, nulls, run in rows[:15]:
        print(f"  {b:6d}  {ms:4d}  {mx:6.1f}  {q:9.3f}  {sn:6.0f}   " + " ".join(f"{v:+5.1f}" for v in worst)
              + f"   {','.join(map(str, nulls)) or '-':>4}  {run:4d}")

if __name__ == "__main__":
    if "--file" in sys.argv:
        analyse(sys.argv[sys.argv.index("--file") + 1])
    else:
        secs = float(sys.argv[sys.argv.index("--secs") + 1]) if "--secs" in sys.argv else 30
        analyse(record(secs))
