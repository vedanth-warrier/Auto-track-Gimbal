"""Clock-sync check. Needs the tdoa_synctest firmware, where bus B's receiver reads bus A's data line.

If the two receivers are clocked and started together, every bus B sample equals the bus A sample
from the same frame. Checks each 2 s segment between forced re-syncs separately, so it also proves
re-syncing keeps alignment. Reports the frame offset between the buses and the fraction of exact matches.
"""
import time
import numpy as np
from tstream import Stream, HOP

REC_S = 12
chunks = []
st = Stream(on_raw=lambda a, b: chunks.append((a.copy(), b.copy())))
time.sleep(1)
chunks.clear()
time.sleep(REC_S)
st.close()

A = np.concatenate([c[0] for c in chunks])   # (frames, 2)
B = np.concatenate([c[1] for c in chunks])
print(f"{len(chunks)} packets, {A.shape[0]} frames ({A.shape[0] / 32000:.1f} s)")
if np.abs(A).max() == 0:
    raise SystemExit("bus A is all zeros: no audio, can't test")


def best_lag(a, b, maxlag=8):
    # frame offset where B matches A best, and the fraction of exactly equal samples there
    best = (None, -1.0)
    for lag in range(-maxlag, maxlag + 1):
        if lag >= 0:
            x, y = a[lag:], b[: len(b) - lag]
        else:
            x, y = a[: len(a) + lag], b[-lag:]
        m = np.mean(np.all(x == y, axis=1))
        if m > best[1]:
            best = (lag, m)
    return best


seg = 250 * HOP   # 2 s between forced re-syncs
results = []
for s in range(0, A.shape[0] - seg // 2, seg):
    a, b = A[s : s + seg], B[s : s + seg]
    # skip the first 2 packets after each boundary (re-sync transients)
    a, b = a[2 * HOP :], b[2 * HOP :]
    if len(a) < HOP:
        continue
    lag, match = best_lag(a, b)
    results.append((lag, match))
    print(f"  segment {len(results):2d}: best frame offset {lag:+d}, exact sample match {100 * match:.2f}%")
lags = {r[0] for r in results}
print(f"\nframe offsets seen: {sorted(lags)}; worst match {100 * min(r[1] for r in results):.2f}%")
print("PASS: buses sample-aligned" if lags == {0} and min(r[1] for r in results) > 0.999 else "FAIL: see above")
