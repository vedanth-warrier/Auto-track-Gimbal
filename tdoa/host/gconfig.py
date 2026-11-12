"""Reads the gimbal settings from tdoa/src/gimbal_config.h; gimbal_cal.py writes the speed tables back."""
import os
import re

_PATH = os.path.join(os.path.dirname(__file__), "..", "src", "gimbal_config.h")
_text = open(_PATH).read()


def _define(name):
    m = re.search(rf"#define\s+GIMBAL_{name}\s+([-0-9.eE]+)f?", _text)
    if not m:
        raise SystemExit(f"GIMBAL_{name} not found in {_PATH}")
    return float(m.group(1))


def _array(name, text=None):
    m = re.search(rf"GIMBAL_{name}\[GIMBAL_CAL_N\]\s*=\s*\{{([^}}]*)\}}", text or _text)
    if not m:
        raise SystemExit(f"GIMBAL_{name} table not found in {_PATH}")
    return [float(v.strip().rstrip("f")) for v in m.group(1).split(",") if v.strip()]


PAN_MIN = _define("PAN_MIN_DEG")
PAN_MAX = _define("PAN_MAX_DEG")
TILT_MIN = _define("TILT_MIN_DEG")
TILT_MAX = _define("TILT_MAX_DEG")
MARGIN = _define("LIMIT_MARGIN_DEG")
TILT_BACKLASH = _define("TILT_BACKLASH_DEG")
P_LO, P_HI = PAN_MIN + MARGIN, PAN_MAX - MARGIN
T_LO, T_HI = TILT_MIN + MARGIN, TILT_MAX - MARGIN
SEARCH_EL = min(max(_define("SEARCH_EL_DEG"), T_LO), T_HI)
SEARCH_PAN = _define("SEARCH_PAN_DEG")
CALIBRATED = bool(_define("CALIBRATED"))
CAL_US = _array("CAL_US")
TABLES = {k: _array(k) for k in ("PAN_FWD_DPS", "PAN_REV_DPS", "TILT_FWD_DPS", "TILT_REV_DPS")}


def dps_for(us, key, tables=None):
    """Speed (deg/s) a speed table gives at pulse offset us: the inverse of offsetFor() in gimbal.cpp, skipping the
    same unusable points, so the board turns it back into exactly this offset. key: e.g. "TILT_FWD_DPS"."""
    pts, pv = [], 0.0
    for u, v in zip(CAL_US, (tables or TABLES)[key]):
        if v > pv:
            pts.append((u, v))
            pv = v
    if not pts:
        return 0.0
    if us <= pts[0][0]:
        return pts[0][1]
    for (u0, v0), (u1, v1) in zip(pts, pts[1:]):
        if us <= u1:
            return v0 + (us - u0) / (u1 - u0) * (v1 - v0)
    return pts[-1][1]


def search_stops():
    return [min(max(p, P_LO), P_HI) for p in (-SEARCH_PAN, 0.0, SEARCH_PAN)]


def write_define(name, value):
    """Rewrite '#define GIMBAL_<name> <value>' in gimbal_config.h, keeping the comment."""
    text = open(_PATH).read()
    text, k = re.subn(rf"(#define\s+GIMBAL_{name}\s+)[-0-9.eE]+f?", lambda m: m.group(1) + f"{value:.1f}f", text)
    if k != 1:
        raise SystemExit(f"GIMBAL_{name} not found in {_PATH}")
    open(_PATH, "w").write(text)


def write_tables(tables):
    """tables: {"PAN_FWD_DPS": [deg/s per CAL_US entry], ...}. Rewrites those lines and sets GIMBAL_CALIBRATED 1."""
    text = open(_PATH).read()
    for name, vals in tables.items():
        body = ", ".join(f"{v:.1f}f" for v in vals)
        text, k = re.subn(rf"(GIMBAL_{name}\[GIMBAL_CAL_N\]\s*=\s*\{{)[^}}]*(\}};)[^\n]*",
                          lambda m: m.group(1) + body + m.group(2), text)
        if k != 1:
            raise SystemExit(f"GIMBAL_{name} table not found in {_PATH}")
    text, k = re.subn(r"(#define\s+GIMBAL_CALIBRATED\s+)\d+", r"\g<1>1", text)
    if k != 1:
        raise SystemExit(f"GIMBAL_CALIBRATED not found in {_PATH}")
    open(_PATH, "w").write(text)
