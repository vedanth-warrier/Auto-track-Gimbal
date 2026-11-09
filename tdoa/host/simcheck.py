"""End-to-end check with the tdoa_sim firmware (synthetic plane wave from a known, moving direction).

Compares the board's direction with the truth in each packet, and with the Python reference run on the
board's own streamed audio (so board maths == reference maths is checked too).
"""
import time
import numpy as np
from tstream import Stream
import tdoa_ref as R

REC_S = 40
rs, blocks = [], []
st = Stream(on_audio=lambda b: blocks.append(b.copy()), on_result=lambda r: rs.append(r))
time.sleep(1)
rs.clear(); blocks.clear()
time.sleep(REC_S)
st.close()
n = min(len(rs), len(blocks))
rs, blocks = rs[:n], blocks[:n]
if not any(r.sim_el for r in rs):
    raise SystemExit("no sim truth in the packets: flash the tdoa_sim firmware")

ang = lambda a, b: np.degrees(np.arccos(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-30), -1, 1)))
valid = [r for r in rs if r.valid]
err = np.array([ang(r.u, R.unit(r.sim_az, r.sim_el)) for r in valid])
print(f"{n} packets ({n / 125:.1f} s), board gave a direction in {len(valid)} ({100 * len(valid) / n:.1f}%), resyncs {rs[-1].resyncs}")
print(f"board vs truth: median {np.median(err):.2f} deg, 99th pct {np.percentile(err, 99):.2f}, max {err.max():.2f}, "
      f"edge flags {sum(r.edge for r in valid)}")
print(f"truth covered el {min(r.sim_el for r in rs):.0f}..{max(r.sim_el for r in rs):.0f} deg, az 0..360")

# reference on the same audio (the stream is 16-bit, the board used 24-bit: tiny differences expected)
T = R.Tracker(); d = []
for b, r in zip(blocks, rs):
    ref = T.step(b)
    if ref is not None and r.valid:
        d.append(ang(ref["u"], r.u))
d = np.array(d)
print(f"board vs Python reference: median {np.median(d):.3f} deg, max {d.max():.3f} deg over {len(d)} packets")
