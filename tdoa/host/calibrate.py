"""Per-mic delay calibration at the tone: removes the constant bias from mic / shroud mismatch.

Setup: tone source (TDOA_TONE_HZ) directly above the array centre, pointing down the array axis, on a stand,
about 30-50 cm up. With the source on the axis every pair delay should be 0; what is measured is each mic's
extra delay plus whatever comes from the source not being exactly centred.
Rounds: rotate the array 90 deg (same direction each time) between rounds, without moving the source.
The source-off-centre part rotates with the array and cancels over 4 rotations; the mics' own delays don't.

  uv run python calibrate.py            measure, print MIC_DELAY_US to paste into src/config.h
  uv run python calibrate.py --write    also write it into src/config.h (then reflash)

Measures from the board's raw audio, so it doesn't depend on the current MIC_DELAY_US.
"""
import os
import re
import sys
import threading
import time
import numpy as np
from tconfig import RATE, TONE_HZ, SPEED, MIC_POS, PAIRS, PAIR_NAMES

ROUNDS = 4
ROUND_S = 10
MIN_SNR_DB = 20       # windows with a weaker tone are skipped
NFFT, HOP = 2048, 256
OMEGA = 2 * np.pi * TONE_HZ
WIN = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(NFFT) / NFFT)
FR = np.fft.rfftfreq(NFFT, 1 / RATE)
NEAR = np.flatnonzero(np.abs(FR - TONE_HZ) <= 150)
FLOOR = np.flatnonzero(np.abs(FR - TONE_HZ) <= 1000)
PD = np.array([MIC_POS[i] - MIC_POS[j] for i, j in PAIRS])
A_MIC = np.array([[1 if m == i else -1 if m == j else 0 for m in range(4)] for i, j in PAIRS], float)


def pair_delays(y):
    """Pair delays t_i - t_j (us) at the tone from audio y (4, n): average of phase-normalised cross-spectra
    over 64 ms windows. Also returns the phase coherence per pair (1 = rock steady) and the median SNR."""
    acc = np.zeros(6, complex); snrs = []
    for s in range(0, y.shape[1] - NFFT, 2 * HOP):
        x = y[:, s : s + NFFT]
        X = np.fft.rfft((x - x.mean(1, keepdims=True)) * WIN, axis=1)
        S = (np.abs(X) ** 2).sum(0)
        k = NEAR[np.argmax(S[NEAR])]
        snr = 10 * np.log10(S[k] / np.median(S[FLOOR]))
        if snr < MIN_SNR_DB:
            continue
        snrs.append(snr)
        g = np.array([(X[i, k - 1 : k + 2] * np.conj(X[j, k - 1 : k + 2])).sum() for i, j in PAIRS])
        acc += g / (np.abs(g) + 1e-30)
    if not snrs:
        return None, None, None
    G = acc / len(snrs)
    return -np.angle(G) / OMEGA * 1e6, np.abs(G), np.median(snrs)


def fit_mic_delays(tau):
    """Per-mic delays d (us, sum 0) with tau_ij ~ d_i - d_j; returns d and the residual per pair."""
    A = np.vstack([A_MIC, np.ones(4)])
    d, *_ = np.linalg.lstsq(A, np.append(tau, 0), rcond=None)
    return d, tau - A_MIC @ d


def source_offset(tau_rot):
    """Off-axis angle and azimuth (array frame) of the source implied by the rotating part of the pair delays."""
    uxy, *_ = np.linalg.lstsq(-PD[:, :2] / SPEED * 1e6, tau_rot, rcond=None)
    r = min(1.0, np.hypot(*uxy))
    return np.degrees(np.arcsin(r)), np.degrees(np.arctan2(uxy[1], uxy[0])) % 360


def write_config(d):
    path = os.path.join(os.path.dirname(__file__), "..", "src", "config.h")
    text = open(path).read()
    new = "static const float MIC_DELAY_US[4] = {" + ", ".join(f"{v:.3f}f" for v in d) + "};"
    text, n = re.subn(r"static const float MIC_DELAY_US\[4\]\s*=\s*\{[^}]*\};", new, text)
    if n != 1:
        raise SystemExit("couldn't find MIC_DELAY_US in config.h; paste it by hand")
    open(path, "w").write(text)
    print(f"written to {os.path.normpath(path)}. Reflash: ~/.platformio/penv/bin/pio run -d tdoa -e tdoa -t upload")


def main():
    from tstream import Stream
    chunks, lock = [], threading.Lock()
    recording = [False]

    def on_audio(b):
        if recording[0]:
            with lock:
                chunks.append(b)

    st = Stream(on_audio=on_audio)
    taus = []
    print(f"tone {TONE_HZ:.0f} Hz. Pairs: {', '.join(PAIR_NAMES)}")
    for r in range(ROUNDS):
        input(f"\nRound {r + 1}/{ROUNDS}: source on the axis above the centre{', array rotated another 90 deg' if r else ''}. "
              f"Enter to record {ROUND_S} s.")
        with lock:
            chunks.clear()
        recording[0] = True
        time.sleep(ROUND_S)
        recording[0] = False
        with lock:
            y = np.hstack(chunks)
        tau, coh, snr = pair_delays(y)
        if tau is None:
            print(f"  no tone above {MIN_SNR_DB} dB SNR: check the tone is {TONE_HZ:.0f} Hz and loud, then redo this round")
            st.close()
            sys.exit(1)
        taus.append(tau)
        print(f"  SNR {snr:.0f} dB, phase steadiness (1 = perfect) " + " ".join(f"{c:.3f}" for c in coh))
        print("  pair delays (us): " + "  ".join(f"{n} {t:+6.2f}" for n, t in zip(PAIR_NAMES, tau)))
        if coh.min() < 0.9:
            print("  WARNING: phase not steady (something moving, or a weak tone); consider redoing this round")
    st.close()

    taus = np.array(taus)
    mean = taus.mean(0)
    d, resid = fit_mic_delays(mean)
    print("\nPer round, source position implied by the rotating part. The source didn't move, so this should be the")
    print("same off-axis angle every round, with the azimuth stepping by about 90 deg:")
    offs = []
    for r, t in enumerate(taus):
        off, az = source_offset(t - mean)
        offs.append(off)
        print(f"  round {r + 1}: source {off:.1f} deg off axis, towards az {az:.0f}")
    if max(offs) - min(offs) > 3:
        print(f"  WARNING: off-axis angle varies by {max(offs) - min(offs):.1f} deg between rounds: the field isn't a clean")
        print("  plane wave (reflections), or the array shifted / the source moved. The result below includes some of that.")
        print("  Refine it with circle.py.")
    print("\nArray-fixed pair delays (us), averaged over rounds: " + "  ".join(f"{n} {t:+6.2f}" for n, t in zip(PAIR_NAMES, mean)))
    print("Per-mic delays (us): " + "  ".join(f"mic{m + 1} {v:+6.2f}" for m, v in enumerate(d)))
    # (no residual check here: with one strong tone the 6 pair phases are always consistent with 4 mic phases)
    print("\nPaste into tdoa/src/config.h:")
    print("static const float MIC_DELAY_US[4] = {" + ", ".join(f"{v:.3f}f" for v in d) + "};")
    if "--write" in sys.argv:
        write_config(d)


if __name__ == "__main__":
    main()
