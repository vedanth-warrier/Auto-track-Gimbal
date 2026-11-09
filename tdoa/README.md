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

## Files
| File | What it does |
|---|---|
| `src/config.h` | Shared settings: tone, speed of sound, **mic positions**, **which I2S slot each mic is wired to** (`MIC_SLOT`), search cone, tracker. The host scripts read this file too. |
| `src/main.cpp` | Firmware: synchronised capture, tone detection, TDOA, direction, streaming. |
| `platformio.ini` | Envs: `tdoa` (tracker), `tdoa_synctest`, `tdoa_sim`, `tdoa_timing` (tests). |
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

## Commands (from the repo root)
```
~/.platformio/penv/bin/pio run -d tdoa -e tdoa -t upload     # flash the tracker
cd tdoa/host
uv run python view.py                                        # or livecheck.py, gcc.py, ...
```
Flash the amplitude tracker again with `~/.platformio/penv/bin/pio run -e mic -t upload` from the repo root.

## Measured
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
- No servo control here: the output is the direction. For a gimbal, drive `ux`, `uy` (array frame) to zero.
