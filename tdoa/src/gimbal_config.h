// Gimbal settings for src/gimbal.cpp (envs gimbal, gimbal_sim). Tone, tracker and mic settings stay in config.h.
// host/gconfig.py reads the GIMBAL_* defines and the speed tables from this file: keep their format.
#pragma once

// ---- Servos (FS90MR continuous rotation) and directions
#define GIMBAL_PAN_PIN 10              // rotation about the vertical
#define GIMBAL_TILT_PIN 11             // elevation
#define GIMBAL_PAN_FWD_SIGN 1          // +1: pan FORWARD (towards mic 4) = pulse above neutral (as servo_gui.py)
#define GIMBAL_TILT_FWD_SIGN 1         // +1: elevation FORWARD (towards mic 2, upwards) = pulse above neutral
#define GIMBAL_UP_AZ_DEG 239.5f        // MIC_POS azimuth of mic 2: elevation FORWARD swings the axis this way
#define GIMBAL_PANFWD_AZ_DEG 329.5f    // MIC_POS azimuth of mic 4: pan FORWARD swings the axis this way
#define GIMBAL_PAN_NEUTRAL_US 1500.0f  // speeds are measured relative to these, so recalibrate if they change
#define GIMBAL_TILT_NEUTRAL_US 1500.0f

// ---- Travel. Home (boot, or "H") = pan centred, plate facing the horizon (elevation 0 = bottom of travel).
// + pan = FORWARD (towards mic 4). Elevation 90 = plate facing straight up.
#define GIMBAL_PAN_MIN_DEG -90.0f
#define GIMBAL_PAN_MAX_DEG 90.0f
#define GIMBAL_TILT_MIN_DEG 0.0f
#define GIMBAL_TILT_MAX_DEG 90.0f
#define GIMBAL_LIMIT_MARGIN_DEG 5.0f   // stay this far inside the limits: the pose is dead-reckoned and drifts
// Elevation worm-gear slack: motor travel (in plate degrees) after a direction change before the plate moves.
// Consistent; measured by host/gimbal_cal.py. Home is zeroed with the slack taken up for moving UP (none to
// move up, all of it to move down).
#define GIMBAL_TILT_BACKLASH_DEG 25.5f

// ---- Tracking
#define GIMBAL_ENABLE 0                // 1 = track from boot; 0 = wait for "E1". Refused until GIMBAL_CALIBRATED
#define GIMBAL_GAIN 2.0f               // deg/s of speed per deg of pose error
#define GIMBAL_MAX_DPS 60.0f           // speed cap while tracking
#define GIMBAL_ACCEL_DPS2 200.0f       // acceleration limit
#define GIMBAL_START_DEG 20.0f         // start moving once the total pointing error exceeds this...
#define GIMBAL_STOP_DEG 20.0f          // ...and stop once it's under this (user 2026-11-12: 20 / 20, rests up to
                                       // ~20 deg off the source; was 10 / 4, before that 4 / 2)
#define GIMBAL_COAST_STOP_S 0.2f       // stop if the tracker has had no usable estimate for this long
#define GIMBAL_EST_LAG_PKTS 0          // 0 = off. Else: rotate each estimate by the gimbal's own motion over this
                                       // many packets (estimate lag: FFT window centre 32 ms + TDOA_CS_TAU 80 ms
                                       // = 14). gimbal_sim 2026-11-12: no benefit at these speeds (all results
                                       // within the START/STOP band either way), and it relies on the speed tables

// ---- Search (tone lost): listen at pan -SEARCH_PAN, 0, +SEARCH_PAN in turn, servos stopped while listening.
// The array hears its whole front hemisphere, so 3 stops cover the travel; full sweeps would add drift.
#define GIMBAL_SEARCH_ENABLE 0         // 0 = hold position when the tone is lost (user's choice 2026-11-12); 1 = search
#define GIMBAL_SEARCH_DELAY_S 1.0f     // s after the track is lost before searching
#define GIMBAL_SEARCH_EL_DEG 30.0f     // elevation of the stops
#define GIMBAL_SEARCH_PAN_DEG 45.0f    // pan of the side stops
#define GIMBAL_SEARCH_DWELL_S 0.6f     // s listening at each stop
#define GIMBAL_SEARCH_DPS 45.0f        // speed between stops
#define GIMBAL_SEARCH_PASSES 2         // passes over the stops (end to end) before parking at the centre

// ---- Speed tables: speed (deg/s) at each pulse offset from neutral, per direction. Measured by
// host/gimbal_cal.py, which rewrites these lines. The pose is speed x time, so it's only as good as these.
#define GIMBAL_CALIBRATED 1
#define GIMBAL_CAL_N 3
static const float GIMBAL_CAL_US[GIMBAL_CAL_N] = {300.0f, 500.0f, 800.0f};  // geared (elevation 5:1, pan less):
                                                                             // usable range 300-800 us (user)
static const float GIMBAL_PAN_FWD_DPS[GIMBAL_CAL_N] = {133.1f, 247.4f, 273.1f};
static const float GIMBAL_PAN_REV_DPS[GIMBAL_CAL_N] = {104.6f, 257.5f, 258.2f};
static const float GIMBAL_TILT_FWD_DPS[GIMBAL_CAL_N] = {61.2f, 118.5f, 159.2f};
static const float GIMBAL_TILT_REV_DPS[GIMBAL_CAL_N] = {71.3f, 211.3f, 222.8f};
