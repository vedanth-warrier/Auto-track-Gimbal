"""Reads the shared settings from tdoa/src/config.h, so the host and the board use the same geometry."""
import os
import re
import numpy as np

_PATH = os.path.join(os.path.dirname(__file__), "..", "src", "config.h")
_text = open(_PATH).read()


def _define(name):
    m = re.search(rf"#define\s+{name}\s+([-0-9.eE]+)f?", _text)
    if not m:
        raise SystemExit(f"{name} not found in {_PATH}")
    return float(m.group(1))


RATE = int(_define("TDOA_RATE"))
TONE_HZ = _define("TDOA_TONE_HZ")
SPEED = _define("TDOA_SPEED")
CONE_DEG = _define("TDOA_CONE_DEG")
TRACK_HOLD_S = _define("TDOA_TRACK_HOLD_S")
TRK_Q_MIN = _define("TDOA_TRK_Q_MIN")
TRK_Q_POW = _define("TDOA_TRK_Q_POW")
TRK_MAX_RATE = _define("TDOA_TRK_MAX_RATE")
TRK_LOST_S = _define("TDOA_TRK_LOST_S")
TRK_TAU = _define("TDOA_TRK_TAU")
DEADBAND_DEG = _define("TDOA_DEADBAND_DEG")
CMD_TAU = _define("TDOA_CMD_TAU")
CS_TAU = _define("TDOA_CS_TAU")
MIN_RUN = int(_define("TDOA_MIN_RUN"))
SERVO_ENABLE = _define("TDOA_SERVO_ENABLE")
PAN_PIN = _define("TDOA_PAN_PIN")
TILT_PIN = _define("TDOA_TILT_PIN")
TILT_UP_AZ_DEG = _define("TDOA_TILT_UP_AZ_DEG")
PAN_SIGN = _define("TDOA_PAN_SIGN")
TILT_SIGN = _define("TDOA_TILT_SIGN")
PAN_NEUTRAL_US = _define("TDOA_PAN_NEUTRAL_US")
TILT_NEUTRAL_US = _define("TDOA_TILT_NEUTRAL_US")
US_PER_RADS = _define("TDOA_US_PER_RADS")
MIN_US = _define("TDOA_MIN_US")
MAX_RATE = _define("TDOA_MAX_RATE")
ACCEL = _define("TDOA_ACCEL")
GAIN = _define("TDOA_GAIN")
START_DEG = _define("TDOA_START_DEG")
STOP_DEG = _define("TDOA_STOP_DEG")
COAST_STOP_S = _define("TDOA_COAST_STOP_S")


def write_defines(values):
    """Rewrite '#define TDOA_<name> <value>' lines in config.h (values: {name: number}), keeping the comments."""
    text = open(_PATH).read()
    for name, v in values.items():
        text, k = re.subn(rf"(#define\s+TDOA_{name}\s+)[-0-9.eE]+f?", lambda m: m.group(1) + (f"{v:.1f}f" if isinstance(v, float) else str(v)), text)
        if k != 1:
            raise SystemExit(f"TDOA_{name} not found in config.h")
    open(_PATH, "w").write(text)

_block = re.search(r"MIC_POS\[4\]\[3\]\s*=\s*\{(.*?)\};", _text, re.S).group(1)
_rows = re.findall(r"\{([^{}]*)\}", _block)
MIC_POS = np.array([[float(v.strip().rstrip("f")) for v in r.split(",") if v.strip()] for r in _rows])
assert MIC_POS.shape == (4, 3), f"MIC_POS in {_PATH} should be 4 rows of 3"

_d = re.search(r"MIC_DELAY_US\[4\]\s*=\s*\{([^}]*)\}", _text)
MIC_DELAY_US = np.array([float(v.strip().rstrip("f")) for v in _d.group(1).split(",")]) if _d else np.zeros(4)
assert MIC_DELAY_US.shape == (4,), f"MIC_DELAY_US in {_PATH} should be 4 values"

_sl = re.search(r"MIC_SLOT\[4\]\s*=\s*\{([^}]*)\}", _text)
MIC_SLOT = [int(v) for v in _sl.group(1).split(",")] if _sl else [1, 0, 3, 2]
assert sorted(MIC_SLOT) == [0, 1, 2, 3], f"MIC_SLOT in {_PATH} must use each slot 0..3 once"

PAIRS = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
PAIR_NAMES = [f"{i + 1}-{j + 1}" for i, j in PAIRS]
PAIR_DIST = np.array([np.linalg.norm(MIC_POS[i] - MIC_POS[j]) for i, j in PAIRS])
