"""Broadband TDOA from sharp sounds (claps, finger snaps, taps on the table), computed on the Mac from the
board's synchronised raw audio. Works with any tdoa firmware build; no tone needed.

A broadband sound has no ghost directions, so this checks the clock sync, the channel order and MIC_POS
with real sound, independent of the tone tracker:
  * clap from a few known directions: the printed azimuth should match
  * clap all the way round the array at about its own height: the largest delay seen per pair approaches
    pair length / speed of sound, so the summary estimates each pair's length to compare with MIC_POS
Per event: GCC-PHAT for each pair (1-12 kHz, 8x interpolated, about 4 us resolution), then the same least
squares as the board. Ctrl-C prints the summary.
"""
import time
import threading
import numpy as np
from tstream import Stream
from tconfig import RATE, SPEED, MIC_POS, PAIRS, PAIR_NAMES, PAIR_DIST

N = 2048              # analysis window (64 ms)
UP = 8                # interpolation factor for the correlation peak
BAND = (1000, 12000)  # Hz used for GCC-PHAT
MAX_LAG_US = 1e6 * PAIR_DIST.max() / SPEED + 150   # search a bit beyond the configured geometry
TRIGGER_DB = 15       # event: short-term high-band energy this far above its running median
PD = np.array([MIC_POS[i] - MIC_POS[j] for i, j in PAIRS])

f = np.fft.rfftfreq(N, 1 / RATE)
band = (f >= BAND[0]) & (f <= BAND[1])
max_lag = int(MAX_LAG_US * 1e-6 * RATE * UP)


def gcc(x):
    X = np.fft.rfft((x - x.mean(1, keepdims=True)) * np.hanning(N), axis=1)
    taus, peaks = [], []
    for i, j in PAIRS:
        C = X[i] * np.conj(X[j])
        C = np.where(band, C / (np.abs(C) + 1e-30), 0)
        c = np.fft.irfft(C, n=N * UP)
        c = np.concatenate([c[-max_lag:], c[: max_lag + 1]])   # lags -max..+max
        k = int(np.argmax(c))
        taus.append((k - max_lag) / (RATE * UP) * 1e6)        # t_i - t_j, us
        peaks.append(c[k] / (np.abs(C).sum() / (N * UP / 2)))  # rough peak sharpness
    return np.array(taus), np.array(peaks)


def direction(tau):
    L = -SPEED * tau * 1e-6                                     # (p_i - p_j).u
    if np.ptp(MIC_POS[:, 2]) < 0.001:
        uxy, *_ = np.linalg.lstsq(PD[:, :2], L, rcond=None)
        r = np.hypot(*uxy)
        uz = np.sqrt(max(0, 1 - r * r)); uxy = uxy / max(1, r)
        u = np.array([uxy[0], uxy[1], uz])
    else:
        u, *_ = np.linalg.lstsq(PD, L, rcond=None); u /= np.linalg.norm(u)
    resid = np.sqrt(np.mean((PD @ u - L) ** 2)) / SPEED * 1e6   # us: how well the 6 delays fit one direction
    return u, resid


def main():
    buf = []
    lock = threading.Lock()

    def on_audio(b):
        with lock:
            buf.append(b.copy())

    st = Stream(on_audio=on_audio)
    print(f"listening. Pair lengths from MIC_POS (mm): " + ", ".join(f"{n} {1000 * d:.0f}" for n, d in zip(PAIR_NAMES, PAIR_DIST)))
    print("  az    el   fit(us)  pair delays t_i - t_j (us): " + " ".join(f"{n:>6}" for n in PAIR_NAMES))
    hist = np.zeros((4, 0)); energy = []; maxabs = np.zeros(6); events = 0; last_event = -1e9
    try:
        while True:
            time.sleep(0.05)
            with lock:
                new = buf[:]; buf.clear()
            if not new:
                continue
            hist = np.hstack([hist] + new)[:, -4 * N:]
            for b in new:   # short-term high-band energy (first difference), per packet
                e = np.sum(np.diff(b, axis=1) ** 2)
                energy.append(e)
            energy = energy[-500:]
            if hist.shape[1] < 2 * N or len(energy) < 50:
                continue
            med = np.median(energy[:-1])
            if 10 * np.log10(energy[-1] / (med + 1e-30)) > TRIGGER_DB and time.time() - last_event > 0.4:
                last_event = time.time()
                time.sleep(0.04)                      # let the rest of the sound arrive
                with lock:
                    hist = np.hstack([hist] + buf); buf.clear()
                x = hist[:, -N:]
                tau, pk = gcc(x)
                u, resid = direction(tau)
                maxabs = np.maximum(maxabs, np.abs(tau)); events += 1
                print(f"{np.degrees(np.arctan2(u[1], u[0])) % 360:4.0f}  {np.degrees(np.arcsin(np.clip(u[2], -1, 1))):4.0f}   {resid:6.1f}   "
                      + " ".join(f"{t:6.0f}" for t in tau))
    except KeyboardInterrupt:
        st.close()
        if events:
            print(f"\n{events} events. Largest |delay| per pair and the pair length it implies (vs MIC_POS):")
            for n, m, d in zip(PAIR_NAMES, maxabs, PAIR_DIST):
                print(f"  {n}: {m:5.0f} us -> at least {1000 * m * 1e-6 * SPEED:5.1f} mm   (MIC_POS: {1000 * d:5.1f} mm)")
            print("Lengths only reach the real value for sounds in line with that pair, at the array's height.")


if __name__ == "__main__":
    main()
