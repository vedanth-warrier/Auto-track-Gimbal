"""Per-mic gain calibration at the tone frequency.

Put the tone source directly above the array centre, pointing down the array axis, playing TONE_HZ.
Each round records ROUND_S seconds; rotate the array ROTATE_DEG between rounds. Position effects
change with rotation, real mic gain differences don't, so the average over rounds is the gain.
Prints a GAIN line to paste into src/mic.cpp.
"""
import threading
import time
import numpy as np
from stream import Stream, RATE

TONE_HZ = 7000     # must match TONE_HZ in src/mic.cpp
ROUNDS = 4         # rotations of the array (90 deg apart covers a full turn)
ROUND_S = 10       # seconds recorded per round
NFFT = 4096

chunks = []        # (4, HOP) audio blocks for the current round
lock = threading.Lock()
recording = False

def on_audio(block):
    if recording:
        with lock:
            chunks.append(block)

Stream(on_audio=on_audio)
win = np.hanning(NFFT)
freqs = np.fft.rfftfreq(NFFT, 1 / RATE)
band = np.where(np.abs(freqs - TONE_HZ) <= 150)[0]

near = np.where(np.abs(freqs - TONE_HZ) <= 1000)[0]

def tone_levels(y):
    # Each mic's tone RMS in dBFS (peak bin of the 4 summed spectra, +/- 2 bins), tone frequency,
    # and SNR: tone vs the median bin within +/- 1 kHz (same idea as the board's detector)
    n = y.shape[1] // NFFT
    P = (np.abs(np.fft.rfft(y[:, : n * NFFT].reshape(4, n, NFFT) * win, axis=2)) ** 2).mean(1)
    S = P.sum(0)
    k = band[np.argmax(S[band])]
    snr = 10 * np.log10(S[k] / np.median(S[near]))
    return 10 * np.log10(2 * P[:, k - 2 : k + 3].sum(1) / (NFFT * (win ** 2).sum())), freqs[k], snr

offsets = []
for r in range(ROUNDS):
    input(f"\nRound {r + 1}/{ROUNDS}: tone playing above the centre{', array rotated 90 deg' if r else ''}? Press Enter to record {ROUND_S} s.")
    with lock:
        chunks.clear()
    recording = True
    time.sleep(ROUND_S)
    recording = False
    with lock:
        y = np.hstack(chunks)
    L, f, snr = tone_levels(y)
    parts = [tone_levels(p)[0] for p in np.array_split(y, 3, axis=1)]
    rel = L - L.mean()
    spread = np.ptp([p - p.mean() for p in parts], axis=0)
    print(f"  tone at {f:.0f} Hz, {L.mean():.1f} dBFS RMS per mic (average), {snr:.0f} dB above the noise nearby")
    print("  offset vs array mean (dB): " + "  ".join(f"mic{m + 1} {rel[m]:+.2f}" for m in range(4)))
    print("  spread within this round:  " + "  ".join(f"{s:.2f}" for s in spread) + " dB")
    if snr < 30:
        print("  WARNING: tone under 30 dB above the noise; turn it up or move it closer, then redo this round")
    offsets.append(rel)

off = np.mean(offsets, axis=0)
sd = np.std(offsets, axis=0)
print("\nAverage offset (dB):        " + "  ".join(f"mic{m + 1} {off[m]:+.2f}" for m in range(4)))
print("Round-to-round std (dB):    " + "  ".join(f"{s:.2f}" for s in sd))
g = 10 ** (-off / 20)
print("\nPaste into src/mic.cpp:")
print("static const float GAIN[4] = {" + ", ".join(f"{v:.3f}f" for v in g) + "};")
