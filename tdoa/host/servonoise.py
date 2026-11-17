"""Does servo noise get into the tone band? Env gimbal (real servos), from tdoa/host.

  uv run python servonoise.py --test 1           beacon OFF: noise near the tone, servos stopped vs running
  uv run python servonoise.py --test 2 --free    beacon ON at a fixed spot: tone SNR and direction scatter

Each servo is run at every offset in OFFSETS (us from neutral) for WINDOW_S, with a stopped window before each, and
the same measurements are made in both. Nothing else drives the servos (tracking stays off).

Test 1 (beacon off): per mic, noise power within +-BAND_HZ of the tone, in dB relative to the stopped windows, plus
how often the board thinks it hears the tone. The servo swings back and forth (SWING_S each way) so the gimbal stays
near where it started: centre each axis when asked.
Test 2 (beacon on, fixed): tone SNR, fit quality and scatter of the raw direction estimate, servos stopped vs running.
The plate must NOT move, or the direction changes for real: take the horn off the servo (or run a spare servo fixed
to the frame) and pass --free; the servo then runs one way the whole window.
Results also go to host/runs/servonoise_<time>.json.
"""
import argparse
import json
import os
import sys
import time
import numpy as np
from gstream import GimbalStream
import tconfig as C

OFFSETS = (300, 500, 800)
WINDOW_S = 3.0
SKIP_S = 0.3              # ignored at the start of each window (servo spin-up, estimator settling)
SWING_S = {300: 0.3, 500: 0.2, 800: 0.15}
BAND_HZ = 1000.0          # the band the board measures its noise floor in
NFFT = 2048

audio, packets = [], []   # (time, block) and (time, Result)
recording = [False]


def on_audio(block):
    if recording[0]:
        audio.append((time.time(), block.copy()))


def on_packet(r, g):
    if recording[0]:
        packets.append((time.time(), r))


def window(st, axis, us, free):
    """Run (or not, us=0) for WINDOW_S; return the audio and packets after SKIP_S."""
    audio.clear()
    packets.clear()
    recording[0] = True
    t0 = time.time()
    sign, last = +1, 0.0
    while time.time() - t0 < WINDOW_S:
        now = time.time()
        if us:
            if not free and now - last >= SWING_S[us]:
                sign, last = -sign, now
            st.send(f"J{axis}{sign * us:+d}")
        time.sleep(0.05)
    recording[0] = False
    st.send("E0")
    time.sleep(0.4)       # servo stops and the room settles before the next window
    keep = t0 + SKIP_S
    a = [b for t, b in audio if t >= keep]
    p = [r for t, r in packets if t >= keep]
    return (np.concatenate(a, axis=1) if a else np.zeros((4, 0))), p


def band_power(x):
    """Mean noise power per mic within +-BAND_HZ of the tone (Welch, Hann, 50 % overlap)."""
    f = np.fft.rfftfreq(NFFT, 1 / C.RATE)
    sel = np.abs(f - C.TONE_HZ) <= BAND_HZ
    w = np.hanning(NFFT)
    out = []
    for m in range(4):
        segs = [x[m, i:i + NFFT] for i in range(0, x.shape[1] - NFFT + 1, NFFT // 2)]
        if not segs:
            out.append(np.nan)
            continue
        ps = np.mean([np.abs(np.fft.rfft((s - s.mean()) * w)) ** 2 for s in segs], axis=0)
        out.append(np.mean(ps[sel]))
    return np.array(out)


def direction_stats(p):
    ok = [r for r in p if r.valid]
    det = len(ok) / max(len(p), 1)
    if len(ok) < 5:
        return det, np.nan, np.nan, np.nan
    u = np.array([r.u for r in ok])
    mean = u.mean(0)
    mean /= np.linalg.norm(mean)
    scatter = np.degrees(np.sqrt(np.mean(np.arccos(np.clip(u @ mean, -1, 1)) ** 2)))
    return det, np.median([r.snr for r in ok]), np.median([r.quality for r in ok]), scatter


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", type=int, choices=[1, 2], required=True)
    ap.add_argument("--free", action="store_true", help="servo not moving the plate (horn off / spare servo)")
    ap.add_argument("--axis", choices=["pan", "tilt", "both"], default="both")
    args = ap.parse_args()
    if args.test == 2 and not args.free:
        sys.exit("Test 2 needs the plate still: take the horn off (or use a spare servo) and pass --free.")
    st = GimbalStream(on_packet=on_packet, on_audio=on_audio)
    time.sleep(0.5)
    st.send("E0")
    axes = [a for a in (("P", "pan"), ("T", "elevation")) if args.axis == "both" or a[1].startswith(args.axis)]
    print(f"Test {args.test}: beacon {'OFF' if args.test == 1 else 'ON, fixed, steady'}. Nobody talking, room as quiet "
          "as it will be in use.\n")
    rows = []
    try:
        for axis, name in axes:
            input(f"== {name} ==  " + ("centre it (mid-travel), " if not args.free else "") + "then Enter")
            for us in OFFSETS:
                for label, u in (("stopped", 0), (f"{us} us", us)):
                    x, p = window(st, axis, u, args.free)
                    det, snr, q, sc = direction_stats(p)
                    rows.append({"axis": name, "cond": label, "us": u, "band": band_power(x).tolist(),
                                 "detect": det, "snr": snr, "quality": q, "scatter": sc})
                    print(f"  {name:9s} {label:8s} recorded")
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
    finally:
        st.send("E0")
        time.sleep(0.1)
        st.close()
    if not rows:
        return
    os.makedirs("runs", exist_ok=True)
    path = time.strftime("runs/servonoise_%Y%m%d_%H%M%S.json")
    json.dump({"test": args.test, "free": args.free, "rows": rows}, open(path, "w"), indent=1)

    print(f"\nTest {args.test} results (saved to {path})")
    if args.test == 1:
        print(f"Noise within +-{BAND_HZ:.0f} Hz of {C.TONE_HZ:.0f} Hz, dB relative to the stopped window just before it;"
              " 'tone' = share of packets where the board reported a tone (beacon off: should be 0)")
        print(f"{'':9s} {'running':8s} {'mic1':>6s} {'mic2':>6s} {'mic3':>6s} {'mic4':>6s} {'mean':>6s} {'tone':>6s}")
        for i in range(1, len(rows), 2):
            off, on = rows[i - 1], rows[i]
            d = 10 * np.log10(np.array(on["band"]) / np.array(off["band"]))
            print(f"{on['axis']:9s} {on['cond']:8s} " + " ".join(f"{v:+6.1f}" for v in d)
                  + f" {10 * np.log10(np.nanmean(on['band']) / np.nanmean(off['band'])):+6.1f} {on['detect']:6.1%}")
        print("\nRead: under ~1 dB = no meaningful servo noise in the band; 3 dB = noise doubled; 'tone' above 0 % = the"
              " servo can trigger false detections. A dead mic shows nan or 0.")
    else:
        print(f"{'':9s} {'cond':8s} {'detect':>7s} {'SNR dB':>7s} {'quality':>8s} {'scatter':>8s}")
        for row in rows:
            print(f"{row['axis']:9s} {row['cond']:8s} {row['detect']:7.1%} {row['snr']:7.1f} {row['quality']:8.3f} "
                  f"{row['scatter']:7.2f}d")
        print("\nRead: compare each running row with the stopped row above it. Falling detect / SNR / quality or rising"
              " scatter while running = the servo noise hurts tracking.")


if __name__ == "__main__":
    main()
