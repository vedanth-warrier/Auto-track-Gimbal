"""Reads the gimbal settings from tdoa/src/gimbal_config.h."""
import os
import re

_PATH = os.path.join(os.path.dirname(__file__), "..", "src", "gimbal_config.h")
_text = open(_PATH).read()


def _define(name):
    m = re.search(rf"#define\s+GIMBAL_{name}\s+([-0-9.eE]+)f?", _text)
    if not m:
        raise SystemExit(f"GIMBAL_{name} not found in {_PATH}")
    return float(m.group(1))


START = _define("START_DEG")
STOP = _define("STOP_DEG")
AXIS_MIN = _define("AXIS_MIN_DEG")
PAN_NEUTRAL = _define("PAN_NEUTRAL_US")
TILT_NEUTRAL = _define("TILT_NEUTRAL_US")
PAN_FWD_SIGN = _define("PAN_FWD_SIGN")
TILT_FWD_SIGN = _define("TILT_FWD_SIGN")
PAN_MIN_US = _define("PAN_MIN_US")
TILT_MIN_US = _define("TILT_MIN_US")
PAN_MAX_US = _define("PAN_MAX_US")
TILT_MAX_US = _define("TILT_MAX_US")
