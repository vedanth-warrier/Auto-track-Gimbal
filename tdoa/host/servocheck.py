"""Checks the board's servo controller against the Python mirror (tdoa_ref.ServoCtl), fed the same tracker output.
Use with tdoa_sim (moving synthetic source). Servos stay off (mode 0): the board still computes and reports the
rates and pulses."""
import time
import numpy as np
from tstream import Stream
import tdoa_ref as R

import glob, serial
_s = serial.Serial(sorted(glob.glob("/dev/cu.usbmodem*"))[0]); _s.dtr = False; _s.rts = True; time.sleep(0.2); _s.rts = False; _s.close()
time.sleep(1.0)   # board reset: its tracker starts from scratch, like the Python one below
rs = []
st = Stream(on_result=lambda r: rs.append(r))
time.sleep(20); st.close()
T = R.DirTracker(); S = R.ServoCtl()
dr, du, dm = [], [], 0
for r in rs:
    state, tu, _ = T.step(r.u if r.valid else None, r.quality)
    pan, tilt, pu, tu_us, moving = S.step(state, T.since, tu if tu is not None else np.zeros(3))
    dr.append(max(abs(pan - r.pan_rate), abs(tilt - r.tilt_rate))); du.append(max(abs(pu - r.pan_us), abs(tu_us - r.tilt_us)))
    dm += moving != r.moving
print(f"{len(rs)} packets; servo mode {rs[-1].servo_mode} (0 = off, nothing driven)")
print(f"board vs Python controller: rates max diff {max(dr):.5f} rad/s, pulses max diff {max(du):.3f} us, moving flag differs {dm} times")
pr = np.array([r.pan_rate for r in rs]); tr = np.array([r.tilt_rate for r in rs])
print(f"rates seen: pan {pr.min():+.2f}..{pr.max():+.2f}, tilt {tr.min():+.2f}..{tr.max():+.2f} rad/s; "
      f"speed max {np.hypot(pr, tr).max():.2f} (cap 1.0); moving {100 * np.mean([r.moving for r in rs]):.0f}% of packets")
acc = np.abs(np.diff(np.hypot(pr, tr))) / (R.HOP / R.RATE)
print(f"largest speed change {acc.max():.2f} rad/s^2 (limit 3.0 per axis)")
