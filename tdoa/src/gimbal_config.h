// Gimbal settings for src/gimbal.cpp (envs gimbal, gimbal_sim). Tone, tracker and mic settings stay in config.h.
// host/gconfig.py reads the GIMBAL_* defines from this file: keep their format.
// Steering uses only the mics: the beacon's angle off the plate axis. No position estimate; travel is limited by
// hard stops on the gimbal.
#pragma once

// ---- Servos (FS90MR continuous rotation) and directions
#define GIMBAL_PAN_PIN 10              // rotation about the vertical
#define GIMBAL_TILT_PIN 11             // elevation
#define GIMBAL_PAN_FWD_SIGN 1          // +1: pan FORWARD (towards mic 4) = pulse above neutral
#define GIMBAL_TILT_FWD_SIGN 1         // +1: elevation FORWARD (towards mic 2, upwards) = pulse above neutral
#define GIMBAL_UP_AZ_DEG 239.5f        // MIC_POS azimuth of mic 2: elevation FORWARD swings the axis this way
#define GIMBAL_PANFWD_AZ_DEG 329.5f    // MIC_POS azimuth of mic 4: pan FORWARD swings the axis this way
#define GIMBAL_PAN_NEUTRAL_US 1500.0f  // stop pulse (only used while moving: a stopped servo gets no pulses)
#define GIMBAL_TILT_NEUTRAL_US 1500.0f

// ---- When to move
#define GIMBAL_ENABLE 0                // 1 = track from boot; 0 = wait for "E1" (gimbal_view.py AUTO)
#define GIMBAL_START_DEG 30.0f         // start moving once the total pointing error exceeds this...
#define GIMBAL_STOP_DEG 30.0f          // ...and stop once it's under this (lower = recentres when it moves)
#define GIMBAL_AXIS_MIN_DEG 5.0f       // while moving, an axis with less error than this stays put (its slowest
                                       // speed would overshoot a small error)
#define GIMBAL_COAST_STOP_S 0.2f       // stop if the tracker has had no usable estimate for this long

// ---- How fast: pulse offset from neutral = MIN + US_PER_DEG x that axis's error, capped at MAX
#define GIMBAL_PAN_MIN_US 300.0f       // slowest offset that reliably moves the axis
#define GIMBAL_PAN_US_PER_DEG 1.0f      // was 4 (user: too fast)
#define GIMBAL_PAN_MAX_US 380.0f       // was 500
#define GIMBAL_TILT_MIN_US 300.0f
#define GIMBAL_TILT_US_PER_DEG 1.0f     // was 4
#define GIMBAL_TILT_MAX_US 380.0f      // was 500

// ---- Look, move, look: the mic direction lags the real one by ~0.15-0.2 s (FFT window + averaging) and the
// servos turn at 60-270 deg/s, so driving continuously overshoots. Each move is a burst of at most MOVE_S, then
// the servos stop for SETTLE_S so the estimate catches up, then it looks again.
#define GIMBAL_MOVE_S 0.08f            // burst length. The estimate is frozen while the servos are noisy, so a burst
                                       // can't stop early on arrival: keep it short (was 0.25 before quiet-only)
#define GIMBAL_SETTLE_S 0.3f           // pause after each burst, counted only while listening to quiet audio

// ---- Quiet-only estimates: servo noise measured +34 to +61 dB within 1 kHz of the tone (servonoise.py,
// 2026-11-17), so the mics are ignored while any servo gets pulses and for QUIET_GAP_S after (the 64 ms FFT window
// must hold only quiet audio). The tracker holds its last direction meanwhile; phase averaging restarts after.
#define GIMBAL_QUIET_GAP_S 0.08f

// ---- Hard stops: no switches, so a stop shows up only as an axis's error not shrinking. If a burst plus its
// settle doesn't reduce an axis's error by STALL_DEG, that direction is blocked (no pulses) until the beacon
// needs the other way. Keeps the servo from pushing on a stop.
#define GIMBAL_STALL_DEG 3.0f          // 0 = off. Only judged on bursts that start with a steady estimate
#define GIMBAL_BLOCK_RETRY_S 3.0f      // a blocked direction is tried again after this (a real stop costs one
                                       // short push per retry; a false block can't stick)
