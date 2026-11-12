# tdoa/ notes

Self-contained TDOA tracker. Do not modify files outside `tdoa/` from work in here (user's rule, 2026-11-04).
See `README.md` for layout, commands and measured results. The parent `CLAUDE.md` working rules still apply.

## Non-obvious facts
- Bus B is synced by routing I2S0's SCK/WS output signals (read back from `GPIO.func_out_sel_cfg[5/6]`) to
  GPIO8/12 (WS moved from GPIO7 2026-11-12) and running I2S1 as an RX slave on those pins. Start order: stop both, start slave, start master.
- I2S1 slave must have its SCK input inverted (`esp_rom_gpio_connect_in_signal(8, I2S1I_BCK_IN_IDX, true)`) and
  its words shifted `<< 1`. Without the inversion it slipped one bit partway through each sample (slip
  position moved with mclk_multiple; 1024x behaved like 384x). With it: 100% bit match in the sync test.
- The sim build writes bus B pre-shifted (`>> 1`) because appendSamples applies the `<< 1`.
- For a planar array a single tone's ghost directions are exact (identical pair phases); they can't be
  detected from the data. Hence the search cone and the "source must start in the cone" rule.
- `host/tconfig.py` parses `src/config.h` with regexes: keep the `#define TDOA_*` and `MIC_POS` formats.
- Packet: "SYNC" + 2 x 1024 B int16 audio + "TDOA" + 19 float32 = 2132 bytes, 125/s.

- `MIC_POS` measured by the user 2026-11-05: 12.4 mm radius, mic 3 at 59.5 deg in its frame (not 0 deg).
  Tone 7.5 kHz chosen from `ambiguity.py --sweep` (ghosts from 8.75 kHz); `TDOA_CONE_DEG` 90 = full sky.

- `MIC_DELAY_US` correction verified 2026-11-05: offline recovery of injected delays exact; on the board (sim with
  injected delays) direction unchanged vs truth (median 1.08 deg), uncorrected it would be 2.8 deg off.
  The sim adds MIC_DELAY_US to its mics, so sim results stay unbiased whatever the calibration.

- calibrate.py run by the user 2026-11-05 -> MIC_DELAY_US {-6.793, 14.315, -18.971, 11.449} (flashed). Per-round
  source offsets were inconsistent (4.7-19.4 deg, az not stepping 90): includes room effects. Its old "residual"
  line was meaningless (one tone: 6 pair phases always fit 4 mic phases) and was removed.
  Livecheck after: much less jumping, but off-axis angle swings with direction (residual bias).
- circle.py: bias = circle centre in (ux, uy); delay change d_m = -1e6 (p_m . b) / c added to MIC_DELAY_US.
  Offline: 2.5 deg bias removed exactly (off-axis 36.8-43.3 -> 40.0 all round). An alternating +,-,+,- delay
  pattern around the ring shifts no direction, so it isn't (and needn't be) corrected.

- circle.py run by the user 2026-11-05 (source 30 deg off axis, continuous rotation by hand): bias 7.5 deg ->
  MIC_DELAY_US {-11.371, 13.256, -17.912, 16.027} (flashed). Scale 1.075 (not acted on: within the uncertainty of
  the source angle). Wobble 6.2 deg rms remains; off-axis seen 12.6-67.1 deg.
- circle.py now has --steps (hands away while recording), saves runs to host/runs/ (gitignored there),
  breaks the wobble into repeats per turn (2x ellipse = MIC_POS axis error, 4x = array-fixed, else reflections)
  and --compare A B for repeatability (correlation of the wobble pattern vs angle).

- Tracker (2026-11-05): board == Python DirTracker exactly (trackcheck, 0.000 deg). tdoa_simbad (fake null on mic 2,
  0.3 s every 5 s): raw up to 53.8 deg off, tracked max 7.9, command max 7.3 (remaining = source moving while the
  tracker coasts blind ~0.5 s). Tuned on replayed board data: Q_MIN 0.95 (clean fits ~0.999), GATE 2.5 deg
  (1.5 rejects good estimates). Settings are TDOA_TRK_* in config.h, read by both board and Python.
- Packet is now 4 + 26 floats (adds track state, tracked u, command u).

- New board 2026-11-07 (identical S3-Zero, MAC ...36:e4), mics soldered. Sync test PASS (100%, offset 0, through
  re-syncs and a reboot). Fresh mics drift in DC for ~15 s after power-up (91% of power < 20 Hz), which made
  broadband RMS/correlation look broken; with < 20 Hz removed all 4 at -50 dBFS, coherent; spikes 0-0.27/s.
- Rewired: mics 1/2 and 3/4 swapped. Slot mapping is now the MIC_SLOT table in config.h (0 A left, 1 A right,
  2 B left, 3 B right), used by the board, the sim and tstream.py. Original wiring = {1, 0, 3, 2}.
- MIC_DELAY_US mostly follows the I2S slot (left-slot mics ~+13 us, right-slot ~-13 us), not the mic. After the
  1/2, 3/4 swap the old values were on the wrong mics: simulated, a source 30 deg off axis read 40-90 deg off,
  pushed along az ~107/287 (between mics 1-3 and 2-4), matching what the user saw. Values moved with the swap
  2026-11-07 (interim); any MIC_SLOT change must move MIC_DELAY_US with it, then recalibrate.
- The main (amplitude) firmware src/mic.cpp and host/stream.py outside tdoa/ still hard-code the old order.

- Servos 2026-11-09: FS90MR continuous rotation, pan GPIO10 (moved from 12 on 2026-11-12: GPIO12 is now bus B WS) / tilt GPIO13 (13 assumed). Velocity-mode controller
  on the board; board == Python ServoCtl exactly on tdoa_sim (rates, pulses, moving flag). Packet 4 + 32 floats.
  Host commands parsed in readCommands(). Not yet calibrated or run on the real gimbal.

- Real recording 2026-11-09 (jumps_20261109_222739, source moved by hand, mostly 45-80 deg off axis): errors are
  slow smooth excursions (10-25 deg, 0.1-1 s), not noise (fast wobble 0.03-0.2 deg; SNR 54-84 dB). 78% coincide
  with a mic 6-20 dB down at the tone: reflection pattern (nulls ~23 mm apart at 7.5 kHz, about the array size).
  Weighting/dropping weak mics barely helped (6.34 -> 6.18 deg). The gated tracker made it worse than raw (coasting
  35-77%, its velocity prediction overshot); stricter gates + heavy smoothing were worse still (51% coasting).
- Tracker replaced: quality floor 0.7, averaging tau 0.1 s weighted by ((q-0.7)/0.3)^2, output speed limit 2 rad/s,
  hold then lost after 0.5 s. Board == Python exactly. Sim nulls: 3.75 / 14.6 deg median/max (unweighted 10.8 / 38).
  Cost: ~6 deg lag at 1 rad/s. No ground truth yet on real data.
- Fundamental fix if excursions stay large near the axis: multi-tone or narrowband-noise beacon (reflection nulls
  land in different places per frequency), or absorbent material near the array.

- Gimbal firmware 2026-11-12: `src/gimbal.cpp` (copy of main.cpp's pipeline + gimbal control; main.cpp untouched,
  envs pick one file via build_src_filter) and `src/gimbal_config.h`. User's spec: pan GPIO10 about the vertical,
  ±90 from centre; elevation GPIO11, 0 = plate facing the horizon (bottom, home) to 90 = facing up (quarter sphere).
  Plate facing the horizon: mic 2 up, mic 3 down, mic 4 / mic 1 at the sides. Pan FORWARD (pulse > 1500, servo_gui
  FORWARD) turns towards mic 4, elevation FORWARD towards mic 2. Tone lost -> search (user's choice: sweep).
  Servos free-spin and pull the wires past the ends (hard stops to come); powered from the laptop's 5 V.
- Aim uses the source's angle relative to the plate, NOT a world-frame target from the dead-reckoned pose: world
  targets flipped over the zenith once the elevation estimate drifted (pan oscillated ±30 deg in the sim).
- Tone lost: holds position (GIMBAL_SEARCH_ENABLE 0, user's choice 2026-11-12). Search, if enabled = 3 listening stops (pan -45/0/+45, el 30), not end-to-end sweeps: full sweeps tripled the drift.
- gimbal_sim results and drift (~4 % of travel with assumed 3-5 % speed errors) in README "Measured".
- No way to sense a stall: the servo's supply doesn't pass through the board. Options: current shunt or INA219
  per servo, or (planned) timed homing into hard stops.
- HWCDC: gimbal.cpp writes only if `!Serial` or availableForWrite() >= packet, so it never blocks with no reader
  (tested: port closed 8 s, 0 re-syncs).

- Elevation worm-gear slack (user 2026-11-12): consistent, only on direction changes; home zeroed with the slack
  taken up for moving UP. Modelled in the dead reckoning (motor side motT, plate moves once outside the gap).
  The elevation speed tables measured before 2026-11-12 crossed the slack on every run (cal alternates
  directions), so they read low: redo with `gimbal_cal.py --axis tilt` (now takes up the slack first).
- Deadband: START 20 / STOP 20 deg on the total pointing error (user 2026-11-12; earlier 10 / 4, 4 / 2).
- Calibration is easy-angle only (user has no protractor): gimbal_cal.py, elevation 0 <-> 90, pan -90 <-> +90;
  first move board-timed (NP/NT command), user fine-adjusts; least squares 90 = k*counted - slack*crossed.
  gimbal_backlash.py and gimbal_travel.py were removed (needed arbitrary angle readings).
- User's calibration 2026-11-12: slowest speeds (300 us) are 93-141 deg/s elevation, 105-133 pan, all above
  GIMBAL_MAX_DPS 60 and the jog's 30: tracking and jogging ONLY use the CAL_US[0] entries. Elevation cal runs were
  0.2-0.65 s with round-number angles; backlash fit down 32 vs up 20 deg (mean 25.5 written). User then saw the
  estimate read ~50 at a true 90 jogging up from home: cause not confirmed (coarse short-run readings most
  likely; GUI resend gaps possible: gimbal_view now counts jog drop-outs). gimbal_travel.py (removed) measured CAL_US[0]
  speeds over the full travel with counted moves. Jog has no limits (user: needed for calibration).

## Open
- Gimbal: gimbal_cal.py --write (easy angles), reflash, then a real-tone test with gimbal_view.py. Not run on hardware yet. Then hard stops + re-homing against them.
- jumps.py recordings: (a) source still on a stand at 3-4 known positions (ground truth), (b) slow movement within
  ~25 deg of the axis (gimbal regime).
- servotest.py --write on the gimbal, reflash, then E1 (auto) with a tone.
- Circle test in step mode, twice in the same place, once moved; --compare to classify the 6 deg wobble.
- Tap test with `host/scope.py`; clap tests with `gcc.py` from known directions.
- Real 7.5 kHz tone test with `view.py` / `livecheck.py`.

- 2026-11-12: mics 3 and 4 (bus B, GPIO4 data) read exactly 0 on both slots; bus A normal (0.3 spikes/s), packets
  125/s, 0 re-syncs (so the bus B clocks reach the I2S1 slave through the GPIO7/8 pads). With GPIO4 pulled up
  (env `tdoa_sdpullup`) bus B read all -1: nothing drives the data line, and GPIO4 is not shorted. Fault is off
  the chip, on what mics 3/4 share: SD wire (GPIO4), SCK (GPIO8) or WS (GPIO7) wire/joint, or their power feed;
  or both mics dead. Happened after the servo tests on this board (wires moved around the header). Unresolved.
  Then: user found the GPIO7 WS joint failed continuity and moved bus B WS to GPIO12 (main.cpp, root mic.cpp;
  TDOA_PAN_PIN moved 12 -> 10). Sync test PASS on GPIO12 (offset 0, 100.00%), so WS reaches the I2S1 slave
  through the pad. Bus B still exactly 0 after that: a second fault remains between the pads and mics 3/4.
