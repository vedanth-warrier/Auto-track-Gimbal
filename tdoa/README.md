# TDOA tracker

Direction finding for the beacon tone (7.5 kHz, `TDOA_TONE_HZ`) from time differences of arrival (TDOA) between the 4 mics, instead of
amplitude. Self-contained: its own PlatformIO project (`platformio.ini`, `src/`) and host scripts (`host/`).
Uses the existing wiring unchanged. Nothing outside this folder is used or modified.

## How it works
- **Synchronised capture.** I2S0 is the clock master on bus A. Its SCK/WS are routed inside the chip (GPIO matrix)
  to bus B's clock pins, and I2S1 runs as a slave on them, so all 4 mics are clocked and sampled on the same
  edges. Both receivers start slave-first on the same frame; a DMA overflow triggers a re-sync.
- **Slave fix.** The slave detects SCK late enough that it sampled the data line as the mics were changing it
  (bits slipped mid-sample). Its SCK input is inverted, which puts its sampling point in the stable half of each
  bit, and the resulting fixed 1-bit lag is undone with `<< 1`. Result: bus B bit-identical to bus A.
- **TDOA at the tone.** 2048-point FFT per mic (64 ms), tone detection as in the amplitude tracker. For each of
  the 6 mic pairs, the cross-spectrum phase at the tone gives the delay (phase = 2 pi f x delay), averaged over
  80 ms.
- **Direction.** A steered search over directions in the upper hemisphere finds the direction whose predicted
  pair phases match best. Least squares on the unwrapped phases then refines it.
- **Ghosts and the search cone.** If mic pairs are long compared with the wavelength, a single tone also fits
  "ghost" directions (for a flat array, exactly). The board can then search only within `TDOA_CONE_DEG` of a
  centre (the array axis until it locks, then the last estimate). With the measured geometry and 7.5 kHz
  there are no ghosts, so `TDOA_CONE_DEG` is 90: the whole upper hemisphere is searched.
- **Mic delay calibration.** `MIC_DELAY_US` (per mic, at the tone) removes constant bias from mic / shroud
  mismatch: each pair's cross-spectrum is rotated by the delay difference before direction finding.
- **Tracker.** Real-room errors are slow reflection excursions, not isolated outliers, so the tracker averages
  the estimates (`TDOA_TRK_TAU`, 0.1 s) weighted by fit quality (`TDOA_TRK_Q_POW`), ignores only clearly broken
  ones (`TDOA_TRK_Q_MIN` 0.7), and caps how fast the output may move (`TDOA_TRK_MAX_RATE`, 2 rad/s: no instant
  jumps). The servo command follows the track with `TDOA_DEADBAND_DEG` of slack. The packet carries raw, tracked
  and command directions.
- **Servos (velocity mode, FS90MR continuous rotation).** Pointing error = tracked direction off the array axis,
  split into tilt (towards `TDOA_TILT_UP_AZ_DEG`, mic 3) and pan. Start moving above `TDOA_START_DEG`, stop
  below `TDOA_STOP_DEG`; speed = `TDOA_GAIN` x error, capped at `TDOA_MAX_RATE`, acceleration-limited; stop if the
  track is lost. Off by default (`TDOA_SERVO_ENABLE`); host commands over USB: E0/E1, TP/TT (test speed),
  NP/NT (stop pulse), S. No position feedback, so no travel limits.
- **Choosing the tone.** `ambiguity.py --sweep` compares frequencies: higher is more precise (error scales with
  1/f), but ghosts appear once pairs get long in wavelengths. For the measured geometry (12.4 mm radius), ghosts
  start at 8.75 kHz; 7.5 kHz keeps the strongest competing direction at a fit of 0.41 (true direction ~1.0).

## Gimbal (`src/gimbal.cpp`, settings `src/gimbal_config.h`)
Separate firmware: `main.cpp`'s pipeline up to the tracker (copied unchanged) driving the FS90MR servos, pan on
GPIO10 (about the vertical) and elevation on GPIO11. Travel is limited by hard stops on the gimbal (pan +-90,
elevation 0 = facing the horizon to 90 = facing up). Steering uses the mics only: no position estimate.
- **Error.** The tracked direction's angle off the plate axis towards mic 2 is the elevation error, towards mic 4
  the pan error. Pan FORWARD (pulse above neutral) swings the axis towards mic 4, elevation FORWARD towards mic 2.
- **Deadband.** Starts once the total error exceeds `GIMBAL_START_DEG`, stops once it's under `GIMBAL_STOP_DEG`
  (both 20); an axis with less than `GIMBAL_AXIS_MIN_DEG` of error stays put.
- **Look, move, look.** The mic estimate lags reality by ~0.15-0.2 s and the servos turn at 60-270 deg/s, so
  continuous driving overshot (sim: pan swung to +77 for a source at +10, then back). Each move is a burst of at
  most `GIMBAL_MOVE_S` (pulse offset = `MIN_US` + `US_PER_DEG` x error, capped at `MAX_US`, per axis), then the
  servos stop for `GIMBAL_SETTLE_S` while the estimate catches up.
- **Hard stops** (no switches): if a burst + settle that started with a steady estimate doesn't shrink an axis's
  error by `GIMBAL_STALL_DEG`, that direction is blocked until the beacon needs the other way, or retried after
  `GIMBAL_BLOCK_RETRY_S`.
- **Quiet-only estimates.** Servo noise measured +34 to +61 dB within 1 kHz of the tone (`servonoise.py`,
  2026-11-17), so while any servo gets pulses and for `GIMBAL_QUIET_GAP_S` after, the tracker is frozen and the
  phase averaging restarts afterwards; the settle counts quiet time only. Bursts can't stop early on arrival, so
  `GIMBAL_MOVE_S` is short (0.15 s).
- **Tone lost:** stop and hold.
- **Packet** = `main.cpp`'s + `"GMBL"` + 18 floats; the TDOA scripts still read it. Never blocks on USB.
- **Host commands:** `E1` auto, `E0`/`S` off, `JP`/`JT<us>` jog at that pulse offset (stops 0.3 s after the last),
  `R` restart the scenario (`gimbal_sim`).

## Files
| File | What it does |
|---|---|
| `src/config.h` | Shared settings: tone, speed of sound, **mic positions**, **which I2S slot each mic is wired to** (`MIC_SLOT`), search cone, tracker. The host scripts read this file too. |
| `src/main.cpp` | Firmware: synchronised capture, tone detection, TDOA, direction, streaming. |
| `src/gimbal.cpp`, `src/gimbal_config.h` | Gimbal firmware and its settings (servos, travel, speed tables, search). |
| `platformio.ini` | Envs: `tdoa` (tracker), `tdoa_synctest`, `tdoa_sdpullup` (bus B wiring check), `tdoa_sim`, `tdoa_timing` (tests), `gimbal`, `gimbal_sim` (closed-loop simulation, servo pins never driven). Each env builds either `main.cpp` or `gimbal.cpp` (`build_src_filter`). |
| `host/view.py` | Live view: direction on a sky plot, pair delays, tone, quality. |
| `host/calibrate.py` | Per-mic delay calibration (source on the axis, 4 rotations); prints or `--write`s `MIC_DELAY_US`. |
| `host/circle.py` | Circle test: source fixed at a known off-axis angle, array rotated a full turn; fits bias (-> `MIC_DELAY_US`), geometry scale and leftover wobble. |
| `host/jumps.py` | Records while it jitters; finds excursions and what coincides with each (quality, SNR, mic null, near-ghost, reacquisition); shows what the tracker makes of it. |
| `host/trackcheck.py` | With `tdoa_simbad`: board tracker vs Python tracker, and raw / tracked / command error vs truth. |
| `host/trackertest.py` | Offline tracker test on a synthetic scenario. |
| `host/servotest.py` | Guided servo calibration: stop pulses, direction signs, speed scale (`--write` to save). |
| `host/servocheck.py` | With `tdoa_sim`: board servo controller vs the Python mirror. |
| `host/scope.py` | 4 scrolling waveforms for tap tests; the loudest tapped mic's title turns red. |
| `host/livecheck.py` | Text summary per second, and board compared with the Python reference. |
| `host/gcc.py` | Broadband TDOA from claps or taps (no tone needed, no ghosts). Checks sync, channel order and geometry with real sound. |
| `host/ambiguity.py` | What your geometry can resolve at the tone; suggests `TDOA_CONE_DEG`. No board needed. |
| `host/synccheck.py` | With `tdoa_synctest`: proves bus A and bus B are sample-aligned. |
| `host/simcheck.py` | With `tdoa_sim`: board vs known synthetic direction, and vs the Python reference. |
| `host/selftest.py` | Offline test of the algorithm on synthetic sources. |
| `host/tdoa_ref.py`, `tstream.py`, `tconfig.py` | Python reference algorithm, serial reader, config reader. |
| `host/gimbal_view.py` | Gimbal: live plot of the beacon relative to the plate axis (from the mics) with the deadband circle, status, AUTO / OFF, hold-to-jog with a pulse-offset slider. |
| `host/servonoise.py` | Servo noise: `--test 1` beacon off, noise near the tone with servos stopped vs running; `--test 2 --free` beacon on, SNR / quality / direction scatter (horn off). |
| `host/gimbal_simcheck.py` | With `gimbal_sim`: one 70 s scripted run; true pointing error per phase vs the best reachable, acquisition, oscillation, time pushing on hard stops. |
| `host/gstream.py`, `gconfig.py` | Gimbal serial reader (TDOA + GMBL blocks), `gimbal_config.h` reader. |

## Commands (from the repo root)
```
~/.platformio/penv/bin/pio run -d tdoa -e tdoa -t upload     # flash the tracker
cd tdoa/host
uv run python view.py                                        # or livecheck.py, gcc.py, ...
```
Gimbal: `~/.platformio/penv/bin/pio run -d tdoa -e gimbal -t upload`, then from `tdoa/host`: `uv run python gimbal_view.py`.

Flash the amplitude tracker again with `~/.platformio/penv/bin/pio run -e mic -t upload` from the repo root.

## Measured
2026-11-15, mic-only gimbal on `gimbal_sim` (true gimbal with hard stops, 6 deg elevation slack, speeds roughly as
measured): true error in each phase's last 4 s: still 13.0, moving source 16.1 median / 21.3 worst, reacquired
elsewhere 11.3, past the pan stop 34.6 (34.4 unavoidable), near zenith 7.9, near horizon 15.6 deg; no reversals;
1.0 s of pushing on stops in 70 s (blocked, retried every ~3.6 s). The first version blocked elevation falsely after
a source jump (estimate still catching up): fixed by judging stops only from bursts that start steady, plus retry.
Entries below about dead reckoning, calibration and slack refer to the earlier, removed position-estimating version.

2026-11-12, frame counting (`gimbal_sim`): N steps of 1 / 2 / 5 frames counted exactly 1.000 / 2.000 / 5.000 frames; a
J jog stopped by E0 counted a whole 5.000. Easy-angle `gimbal_cal.py` with one-pulse fine steps, scripted user on the
sim: every speed within 3 % of the sim's error, slack 7.0 vs 6, no redos. Not checked on the real pin: that a new
duty starts exactly at the next timer overflow is from the ESP-IDF LEDC documentation.

2026-11-12, `gimbal_sim`, easy-angle `gimbal_cal.py` driven by a script watching the sim's true gimbal (sim errors:
elevation up -5 %, down +5 %, slack 6; pan forward +5 %, reverse -3 %): recovered up -4 / -6 / -7 %, down +5 / +4 / +5 %,
slack 5.1 deg, pan forward +6 / +6 / +4 %, reverse -3 % at all offsets; stopping tolerance 1.2 deg. Found and fixed
first: stopping by Enter overshot by up to 0.1 s (the repeat thread), and at 500-800 us a whole run takes ~0.4 s, so
the first move is now board-timed. Deadband 20 / 20: rests 10-17 deg off a still source.

2026-11-12, `gimbal_sim` with 6 deg of true elevation slack and no speed errors (isolates the slack model):
estimate vs true elevation, worst / end of run: model off 6.6 / -6.0 deg, model on 0.6 / 0.0 deg. Pointing error
near the horizon 11.5 -> 6.6 deg (3.0 unavoidable at the margin, up to 4 from the stop band). Scripted test of
the (since removed) `gimbal_backlash.py` on a plate model (slack 6, reading noise 0.3 deg): 6.2 down, 6.1 up.

2026-11-12, `gimbal_sim` (board's real DSP and tracker; simulated array on the gimbal's true pose; servo speed
errors +5 % / -3 % pan, -5 % / +5 % elevation, an assumed residual calibration error, not measured):
- Still source: acquired in 0.21 s, settled 0.8 deg true error, 0 reversals. Source moving 9 deg/s: median 4.6,
  worst 6.9 deg (start/stop band plus lag).
- Tone back on 125 deg away (behind the plate, so not heard): found by the search in 1.6 s, settled 0.6 deg.
- Source behind (az 150): found at a side stop, tracked to the pan limit (39 deg error, 52 unavoidable at the margin).
- Near the zenith (el 82): 9.7 deg, because the elevation estimate had drifted +13 deg into its limit. Near the
  horizon (el 2): 3.2 deg (3.0 unavoidable, margin).
- Drift after 70 s: pan -28, elevation +15 deg, with ~690 deg of pan travel: about 4 % of travel, as the speed
  errors predict. The true pan reached 103.5 (13.5 past the end) though the estimate stayed inside. Before the
  3-stop search and the relative aim, the same run drifted 73 deg (full sweeps, and a pan oscillation near the
  zenith from world-frame targets).
- Lag compensation of the estimates (`GIMBAL_EST_LAG_PKTS` 14) and turning the tracker with the plate: all
  results stayed within the start/stop band either way (still 2.4 / 0.8 / 3.7 deg final error for lag 14 / lag 0 /
  neither). Kept: tracker turning; lag compensation off.

2026-11-05, measured geometry (12.4 mm radius), 7.5 kHz, full-sky search:
- `ambiguity.py`: no ghosts anywhere in the hemisphere; strongest competing direction fit 0.41.
  Precision near the axis 0.84 deg rms per 0.05 rad of pair phase noise (7 kHz: 0.90; 8 kHz: 0.79 but fit 0.62).
- `selftest.py`: still sources from the axis down to 10 deg elevation, median / worst error 0.01 / 0.07 deg at
  34 dB SNR, 0.07 / 0.74 deg at 14 dB, 0.20 / 2.71 deg at 4 dB. Sources up to 60 deg off axis acquired from
  the axis at every azimuth. Moving source tracked, median 1.09 deg (averaging lag).
- `tdoa_sim` on the board: vs truth median 1.08 deg, worst 1.48 deg; vs Python reference worst 0.139 deg.
- Live: 125.0 packets/s, 0 re-syncs, 0 false tone outputs in 30 s of room sound.

2026-11-04, placeholder geometry (40 mm radius), 7 kHz:
- Sync (`tdoa_synctest`): bus B bit-identical to bus A, 100.00% of samples, frame offset 0, across forced
  re-syncs every 2 s and 2 hard reboots.
- Timing (`tdoa_timing`): 4.84 ms per 8 ms packet (worst 4.86 ms); 125.0 packets/s; 0 re-syncs in 60 s live.
- Sim (`tdoa_sim`, source moving az 20 deg/s, el 90 -> 45 -> 90): board vs truth median 1.08 deg, worst 1.46 deg
  (lag from the 80 ms averaging); board vs Python reference worst 0.135 deg.
- Offline (`selftest.py`): still sources in the cone, worst error 0.17 deg even at 4 dB SNR; sources starting
  25 deg off axis found from the axis; at 30 deg half lock onto a ghost, beyond that all do.
- Real sound (`gcc.py`, room sounds): 6 pair delays consistent with one direction to 1.4-3.6 us, including the
  4 pairs that cross buses. Confirms sync and channel order with real sound.
- False tone outputs on room sound: 11 of 7500 packets (0.15%).

## Limits
- Azimuth is measured from `MIC_POS`'s own +x axis (mic 3 at 59.5 deg), not from mic 3.
- The array is small (25 mm across), so precision depends heavily on SNR: keep the beacon well above the room
  noise. Real rooms add phase errors from reflections; not yet measured.
- A wrong `MIC_POS` scale still fits one direction, just at the wrong elevation: check with `gcc.py`.
- Changing `MIC_POS` or the tone: rerun `ambiguity.py` (and `--sweep`) and set `TDOA_CONE_DEG` from it.
- Elevation is weakest near the axis (a flat array measures the in-plane part of the direction); azimuth and
  the off-axis components `ux`, `uy` are the reliable outputs for steering towards the source.
- `main.cpp`'s own servo mode (`E1`, pins 10 / 13) predates `gimbal.cpp` and isn't maintained; use `gimbal.cpp`.
- Gimbal pose is dead-reckoned: drift grows with degrees travelled (sim: ~4 % with 3-5 % speed errors). Nothing
  corrects it until hard stops are fitted and it re-homes against them.
