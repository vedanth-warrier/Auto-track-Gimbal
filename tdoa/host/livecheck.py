"""Text check of the live tracker: stream health, then once per second the tone and direction statistics,
and the board compared with the Python reference on the same audio. Play the tone and move it around.
Ctrl-C to stop.
"""
import time
import numpy as np
from tstream import Stream
import tdoa_ref as R

rs, blocks = [], []
st = Stream(on_audio=lambda b: blocks.append(b.copy()), on_result=lambda r: rs.append(r))
T = R.Tracker()
ang = lambda a, b: np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1)))
print("sec  pkts  tone%   SNR   az    el   spread  quality  ambig  edge%  board-vs-ref | track% coast%  tracked-spread  cmd-moves  resyncs")
t0 = time.time()
try:
    while True:
        time.sleep(1)
        n = min(len(rs), len(blocks))
        cur, blk = rs[:n], blocks[:n]
        del rs[:n], blocks[:n]
        refs = [T.step(b) for b in blk]
        v = [r for r in cur if r.valid]
        if not cur:
            print("no packets: is the tdoa firmware flashed?")
            continue
        if v:
            U = np.array([r.u for r in v]); m = U.mean(0); m /= np.linalg.norm(m)
            spread = np.median([ang(u, m) for u in U])
            d = [ang(ref["u"], r.u) for ref, r in zip(refs, cur) if ref is not None and r.valid]
            tr = [r.tu for r in cur if r.track_state]
            if tr:
                tm = np.mean(tr, 0); tm /= np.linalg.norm(tm)
                tspread = np.median([ang(t, tm) for t in tr])
            else:
                tspread = float("nan")
            cm = [r.cmd for r in cur if np.any(r.cmd)]
            cmdmove = max(ang(a, b) for a in cm for b in cm[-1:]) if cm else float("nan")   # how far the command moved this second
            print(f"{time.time() - t0:3.0f}  {len(cur):4d}  {100 * len(v) / len(cur):4.0f}  {np.median([r.snr for r in v]):5.0f}  "
                  f"{np.degrees(np.arctan2(m[1], m[0])) % 360:4.0f}  {np.degrees(np.arcsin(m[2])):4.0f}  {spread:6.2f}  "
                  f"{np.median([r.quality for r in v]):7.2f}  {np.median([r.ambiguity for r in v]):5.2f}  "
                  f"{100 * np.mean([r.edge for r in v]):5.0f}  {np.max(d) if d else float('nan'):10.3f}   | "
                  f"{100 * np.mean([r.track_state == 1 for r in cur]):5.0f}  {100 * np.mean([r.track_state == 2 for r in cur]):5.0f}  "
                  f"{tspread:13.2f}  {cmdmove:8.2f}    {cur[-1].resyncs}")
        else:
            print(f"{time.time() - t0:3.0f}  {len(cur):4d}     0   (no tone detected)                                       {cur[-1].resyncs}")
except KeyboardInterrupt:
    st.close()
