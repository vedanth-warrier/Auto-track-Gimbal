// Shared settings for the TDOA tracker. The host scripts (tdoa/host/tconfig.py) read the
// TDOA_* defines and MIC_POS below straight from this file, so keep their format.
#pragma once

#define TDOA_RATE 32000          // Hz sample rate
#define TDOA_TONE_HZ 7500.0f     // beacon frequency: highest with a wide ghost margin for MIC_POS (ambiguity.py --sweep)
#define TDOA_SPEED 343.0f        // m/s speed of sound (about 20 C)

// Search cone (deg). If mic pairs are long compared with the wavelength, a single tone's phases also fit
// "ghost" directions; the board then searches only within this angle of a centre (the array axis until it
// locks, then its last estimate). 90 = search the whole upper hemisphere (no ghosts for this MIC_POS at
// 7.5 kHz: strongest competing direction fits 0.41). Rerun host/ambiguity.py after changing MIC_POS or the tone.
#define TDOA_CONE_DEG 90.0f
#define TDOA_TRACK_HOLD_S 0.5f   // s without a tone before the cone recentres on the array axis

// Direction tracker: turns the raw estimates into a smooth track and a servo command. (Replaced the gated
// tracker 2026-11-09: on real data its gates rejected good estimates and made it worse than raw.)
#define TDOA_TRK_Q_MIN 0.70f        // estimates with a worse fit quality (0..1) are ignored (only clearly broken ones)
#define TDOA_TRK_Q_POW 2.0f         // averaging weight ((quality - Q_MIN) / (1 - Q_MIN))^this: weaker fits count less
#define TDOA_TRK_MAX_RATE 2.0f      // rad/s: the tracked direction can't move faster than this (no instant jumps)
#define TDOA_TRK_LOST_S 0.5f        // s without an accepted estimate before the track is dropped
#define TDOA_TRK_TAU 0.1f           // s, averaging of the estimates (lag ~ source speed x TAU: ~6 deg at 1 rad/s)
#define TDOA_DEADBAND_DEG 1.5f      // servo command only moves once the track is this far from it
#define TDOA_CMD_TAU 0.0f           // s, extra smoothing of the servo command after the deadband (0 = off)
#define TDOA_CS_TAU 0.08f           // s, averaging of the pair phases before direction finding
#define TDOA_MIN_RUN 6              // packets (8 ms each) the tone must be held before it counts

// Which I2S slot each mic's data arrives in (mic 1..4): 0 = bus A left, 1 = bus A right, 2 = bus B left,
// 3 = bus B right. Left = L/R pin to GND, right = L/R pin to 3V3. Rewired 2026-11-07 (new board): mics 1/2 and
// 3/4 swapped relative to the original {1, 0, 3, 2}. host/scope.py tap test checks it.
static const int MIC_SLOT[4] = {0, 1, 2, 3};

// Acoustic position of each mic in metres: where sound enters its shroud (mouth centre).
// Measured 2026-11-05: mouths on a 12.4 mm radius circle, 90 deg apart, in one plane. Azimuth is measured
// from this frame's +x axis (mic 3 is at 59.5 deg, mic 1 at 149.5, mic 2 at 239.5, mic 4 at 329.5);
// +z = array axis (up). host/gcc.py (claps) can check them.
static const float MIC_POS[4][3] = {
    {-0.01071f, 0.0063f, 0.000f},    // mic 1 (faces 90)
    {-0.0063f, -0.01071f, 0.000f},   // mic 2 (faces 180)
    {0.0063f, 0.01071f, 0.000f},    // mic 3 (faces 0)
    {0.01071f, -0.0063f, 0.000f},   // mic 4 (faces 270)
};

// Per-mic extra delay (us) at TDOA_TONE_HZ from mic / shroud mismatch, measured by host/calibrate.py (source on
// the axis, 4 rotations). Removed before direction finding. Only differences between mics matter (sum ~0).
// Phase-based: valid at the tone it was measured at; recalibrate if the tone changes.
// Most of each value follows the I2S SLOT, not the mic (left-slot mics read ~+13 us, right-slot ~-13 us), so if
// MIC_SLOT changes these must move with it, then recalibrate. 2026-11-07: moved with the 1/2 and 3/4 swap.
static const float MIC_DELAY_US[4] = {13.960f, -11.869f, 16.525f, -18.616f};

// ---- Servos: FS90MR continuous rotation (pulse sets speed: ~1500 us = stop). No position feedback, so no
// travel limits: the board stops them whenever the track is lost. Calibrate with host/servotest.py.
#define TDOA_SERVO_ENABLE 0          // 1 = drive the servos at boot. 0 = compute and report only (enable with "E1")
#define TDOA_PAN_PIN 12
#define TDOA_TILT_PIN 13
#define TDOA_TILT_UP_AZ_DEG 59.5f    // array-frame direction that tilting up moves the pointing towards (mic 3)
#define TDOA_PAN_SIGN 1              // +1 / -1: flip if that servo turns away from the source (servotest.py checks)
#define TDOA_TILT_SIGN 1
#define TDOA_PAN_NEUTRAL_US 1500.0f  // pulse at which each servo stands still (servotest.py trims it)
#define TDOA_TILT_NEUTRAL_US 1500.0f
#define TDOA_US_PER_RADS 70.0f       // pulse offset per rad/s of gimbal speed (servotest.py measures it)
#define TDOA_MIN_US 0.0f             // extra offset to get past the servo's own deadband
#define TDOA_MAX_RATE 1.0f           // rad/s, speed cap
#define TDOA_ACCEL 3.0f              // rad/s^2, acceleration limit
#define TDOA_GAIN 2.0f               // rad/s of speed per rad of pointing error
#define TDOA_START_DEG 3.0f          // start moving once the error exceeds this...
#define TDOA_STOP_DEG 1.5f           // ...and stop once it's back under this (hysteresis)
#define TDOA_COAST_STOP_S 0.2f       // stop if the tracker has been coasting this long (or lost the source)
