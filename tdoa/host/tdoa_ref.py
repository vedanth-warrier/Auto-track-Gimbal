"""Python reference of the board's TDOA algorithm (keep in sync with tdoa/src/main.cpp).

Used by selftest.py (synthetic sources), ambiguity.py (what the geometry can resolve) and
boardcheck.py (board vs reference on the same recorded audio).
"""
import numpy as np
from tconfig import RATE, TONE_HZ, SPEED, MIC_POS, PAIRS, CONE_DEG, TRACK_HOLD_S, MIC_DELAY_US

HOP = 256
NFFT = 2048
BIN_HZ = RATE / NFFT
OMEGA = 2 * np.pi * TONE_HZ
TONE_TOL_HZ, FLOOR_HZ, DETECT_DB = 150, 1000, 12
from tconfig import MIN_RUN, CS_TAU
AZ_STEP, EL_STEP, AMBIG_SEP = 4, 4, 30
K_TOL = max(2, int(TONE_TOL_HZ / BIN_HZ + 0.5))
K_FLOOR = int(FLOOR_HZ / BIN_HZ + 0.5)
NR = 2 * K_FLOOR + 1
K0 = int(round(TONE_HZ / BIN_HZ)) - K_FLOOR
WIN = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(NFFT) / NFFT)   # ESP-DSP's Hann
PD = np.array([MIC_POS[i] - MIC_POS[j] for i, j in PAIRS])              # (6, 3)
PLANAR = np.ptp(MIC_POS[:, 2]) < 0.001
CORR = np.exp(1j * OMEGA * 1e-6 * np.array([MIC_DELAY_US[i] - MIC_DELAY_US[j] for i, j in PAIRS]))   # removes mic delays


def grid():
    us = []
    for e in range(int(90 / EL_STEP + 0.5) + 1):
        el = 90.0 if e == int(90 / EL_STEP + 0.5) else e * EL_STEP
        for a in range(1 if el == 90.0 else int(360 / AZ_STEP + 0.5)):
            az = a * AZ_STEP
            us.append([np.cos(np.radians(el)) * np.cos(np.radians(az)),
                       np.cos(np.radians(el)) * np.sin(np.radians(az)), np.sin(np.radians(el))])
    return np.array(us)


GU = grid()                                                  # (NG, 3)
THETA = OMEGA * (GU @ PD.T) / SPEED                          # (NG, 6) predicted pair phases
STEER = np.cos(THETA) + 1j * np.sin(THETA)


def plane_wave(u, n, amp=0.01, noise=0.0, rng=None, t0=0, delays_us=MIC_DELAY_US):
    """n samples per mic of a tone arriving from unit vector u (array frame), optional white noise.
    Each mic also gets its extra delay (default: the configured MIC_DELAY_US, as real mics would have)."""
    t = (t0 + np.arange(n)) / RATE
    off = OMEGA * (MIC_POS @ u) / SPEED - OMEGA * 1e-6 * np.asarray(delays_us)   # mic i hears sin(w (t - t_i))
    x = amp * np.sin(OMEGA * t[None, :] + off[:, None])
    if noise:
        x = x + noise * (rng or np.random.default_rng()).standard_normal(x.shape)
    return x


def unit(az_deg, el_deg):
    az, el = np.radians(az_deg), np.radians(el_deg)
    return np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])


def wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def solve(G, centre=(0, 0, 1), cone_deg=CONE_DEG):
    """Direction from averaged phase-normalised cross-spectra G (6 complex), best match taken within cone_deg
    of centre. Returns a dict like the board's packet, or None if the cone holds no grid point."""
    P = (G[None, :] * np.conj(STEER)).real.sum(1)
    dots = GU @ np.asarray(centre, float)
    inside = dots >= np.cos(np.radians(cone_deg)) if cone_deg < 90 else np.ones(len(GU), bool)
    if not inside.any():
        return None
    best = int(np.flatnonzero(inside)[np.argmax(P[inside])])
    edge = cone_deg < 90 and dots[best] < np.cos(np.radians(cone_deg - max(AZ_STEP, EL_STEP)))
    far = GU @ GU[best] < np.cos(np.radians(AMBIG_SEP))
    p2 = P[far].max() if far.any() else 0
    w = np.abs(G)
    th = THETA[best]
    L = (th + wrap(np.angle(G) - th)) * SPEED / OMEGA       # (p_i - p_j).u
    if PLANAR:
        A = (PD[:, :2] * w[:, None]).T @ PD[:, :2]
        b = (PD[:, :2] * w[:, None]).T @ L
        uxy = np.linalg.solve(A, b) if abs(np.linalg.det(A)) > 1e-12 else GU[best, :2]
        r = np.hypot(*uxy)
        if r > 1:
            uxy = uxy / r
            r = 1
        u = np.array([uxy[0], uxy[1], np.sqrt(1 - r * r)])
    else:
        A = (PD * w[:, None]).T @ PD
        b = (PD * w[:, None]).T @ L
        u = np.linalg.solve(A, b) if abs(np.linalg.det(A)) > 1e-15 else GU[best].copy()
        u = u / (np.linalg.norm(u) + 1e-30)
    amb = p2 / P[best] if p2 > 0 else 0
    return dict(u=u, az=np.degrees(np.arctan2(u[1], u[0])) % 360, el=np.degrees(np.arcsin(np.clip(u[2], -1, 1))),
                quality=P[best] / (w.sum() + 1e-30), ambiguity=amb, edge=bool(edge),
                tau=-1e6 * L / SPEED, grid_best=best)


class Tracker:
    """Packet-by-packet reference: feed (4, HOP) blocks in full-scale units."""

    def __init__(self):
        self.ring = np.zeros((4, NFFT))
        self.fill = 0
        self.run = 0
        self.last_freq = 0
        self.G = None
        self.a = 1 - np.exp(-HOP / RATE / CS_TAU) if CS_TAU > 0 else 1
        self.centre = np.array([0.0, 0.0, 1.0])
        self.packets = 0
        self.last_fix = 0

    def step(self, block):
        self.packets += 1
        if self.packets - self.last_fix > int(TRACK_HOLD_S * RATE / HOP):
            self.centre = np.array([0.0, 0.0, 1.0])
        self.ring = np.roll(self.ring, -HOP, axis=1)
        self.ring[:, -HOP:] = block
        self.fill = min(self.fill + HOP, NFFT)
        if self.fill < NFFT:
            return None
        x = (self.ring - self.ring.mean(1, keepdims=True)) * WIN
        X = np.fft.fft(x, axis=1)[:, K0 : K0 + NR]
        S = (np.abs(X) ** 2).sum(0)
        self.X, self.S = X, S          # internals, for diagnostics (jumps.py)
        c = K_FLOOR
        k = c - K_TOL + int(np.argmax(S[c - K_TOL : c + K_TOL + 1]))
        rest = np.delete(S, np.arange(max(0, k - 4), min(NR, k + 5)))
        snr = 10 * np.log10(S[k] / (np.partition(rest, len(rest) // 2)[len(rest) // 2] + 1e-30))
        self.k, self.snr = k, snr
        hit = snr >= DETECT_DB
        if hit:
            a, b, cc = np.log(S[k - 1] + 1e-30), np.log(S[k] + 1e-30), np.log(S[k + 1] + 1e-30)
            den = a - 2 * b + cc
            freq = (K0 + k + (0.5 * (a - cc) / den if den != 0 else 0)) * BIN_HZ
            if self.run > 0 and abs(freq - self.last_freq) > 2 * BIN_HZ:
                self.run = 0
            self.last_freq = freq
        self.run = self.run + 1 if hit else 0
        if self.run < MIN_RUN:
            if self.run == 0:
                self.G = None
            return None
        g = np.array([(X[i, k - 1 : k + 2] * np.conj(X[j, k - 1 : k + 2])).sum() for i, j in PAIRS])
        g = g / (np.abs(g) + 1e-30) * CORR
        self.g = g                     # this packet's phase-normalised cross-spectra (before averaging)
        self.G = g if self.G is None else self.G + self.a * (g - self.G)
        r = solve(self.G, self.centre)
        if r is None:
            return None
        self.centre = r["u"].copy()
        self.last_fix = self.packets
        r.update(freq=freq, snr=snr)
        return r


# ---------------------------------------------------------------- direction tracker (mirrors main.cpp)
from tconfig import TRK_Q_MIN, TRK_Q_POW, TRK_MAX_RATE, TRK_LOST_S, TRK_TAU, DEADBAND_DEG, CMD_TAU


def _norm(v):
    return v / (np.linalg.norm(v) + 1e-30)


class DirTracker:
    """Per packet: step(u or None, quality) -> (state, tracked u, command u). state 0 lost, 1 tracking, 2 holding.
    Averages usable estimates (TRK_TAU), caps the output speed (TRK_MAX_RATE), ignores quality < TRK_Q_MIN."""
    DT = HOP / RATE

    def __init__(self):
        self.state = 0
        self.e = np.array([0.0, 0, 1]); self.o = self.e.copy()
        self.since = 0.0
        self.cmd = None; self.cmd_s = None

    def step(self, u, quality):
        dt = self.DT
        alpha = 1 - np.exp(-dt / TRK_TAU)
        max_step = np.degrees(TRK_MAX_RATE * dt)
        if u is not None and quality >= TRK_Q_MIN:
            u = np.asarray(u, float)
            if self.state == 0:
                self.e = u.copy(); self.o = u.copy()
            else:   # weaker fits count less
                w = ((quality - TRK_Q_MIN) / (1 - TRK_Q_MIN)) ** TRK_Q_POW
                self.e = _norm(self.e + alpha * w * (u - self.e))
            self.state = 1; self.since = 0.0
        elif self.state:
            self.since += dt
            self.state = 0 if self.since >= TRK_LOST_S else 2
        if self.state:
            a = np.degrees(np.arccos(np.clip(self.o @ self.e, -1, 1)))
            f = max_step / a if a > max_step else 1.0
            self.o = _norm(self.o + f * (self.e - self.o))
            if self.cmd is None:
                self.cmd = self.o.copy(); self.cmd_s = self.o.copy()
        t = self.o.copy() if self.state else None
        if t is not None:
            a = np.degrees(np.arccos(np.clip(self.cmd @ t, -1, 1)))
            if a > DEADBAND_DEG:
                self.cmd = _norm(self.cmd + (1 - DEADBAND_DEG / a) * (t - self.cmd))
        if self.cmd is not None:
            b = 1 - np.exp(-dt / CMD_TAU) if CMD_TAU > 0 else 1
            self.cmd_s = _norm(self.cmd_s + b * (self.cmd - self.cmd_s))
        return self.state, t, (self.cmd_s.copy() if self.cmd_s is not None else None)


# ---------------------------------------------------------------- servo controller (mirrors main.cpp servoStep)
from tconfig import (TILT_UP_AZ_DEG, PAN_SIGN, TILT_SIGN, PAN_NEUTRAL_US, TILT_NEUTRAL_US, US_PER_RADS, MIN_US,
                     MAX_RATE, ACCEL, GAIN, START_DEG, STOP_DEG, COAST_STOP_S)


def rate_to_us(rate, neutral, sign):
    off = sign * rate * US_PER_RADS
    if abs(rate) > 1e-3:
        off += np.sign(off) * MIN_US
    return neutral + off


class ServoCtl:
    """Automatic mode, per packet: step(track_state, since, tracked u) -> (pan, tilt rad/s, pan_us, tilt_us, moving)."""

    def __init__(self):
        self.moving = False
        self.pan = self.tilt = 0.0
        a = np.radians(TILT_UP_AZ_DEG)
        self.up = np.array([np.cos(a), np.sin(a)])
        self.right = np.array([self.up[1], -self.up[0]])   # 90 deg clockwise from up

    def step(self, state, since, tu):
        tp = tt = 0.0
        if state == 1 or (state == 2 and since < COAST_STOP_S):
            e = np.asarray(tu[:2], float); r = np.hypot(*e)
            err = np.arcsin(min(1.0, r))
            if not self.moving and err > np.radians(START_DEG):
                self.moving = True
            if self.moving and err < np.radians(STOP_DEG):
                self.moving = False
            if self.moving and r > 0:
                s = min(MAX_RATE, GAIN * (err - np.radians(STOP_DEG)))
                tt = s * (e @ self.up) / r
                tp = s * (e @ self.right) / r
        else:
            self.moving = False
        d = ACCEL * HOP / RATE
        self.pan += np.clip(tp - self.pan, -d, d)
        self.tilt += np.clip(tt - self.tilt, -d, d)
        return (self.pan, self.tilt, rate_to_us(self.pan, PAN_NEUTRAL_US, PAN_SIGN),
                rate_to_us(self.tilt, TILT_NEUTRAL_US, TILT_SIGN), self.moving)

