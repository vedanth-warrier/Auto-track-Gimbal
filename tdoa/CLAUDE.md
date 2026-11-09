# tdoa/ notes

Self-contained TDOA tracker. Do not modify files outside `tdoa/` from work in here (user's rule, 2026-11-04).
See `README.md` for layout, commands and measured results. The parent `CLAUDE.md` working rules still apply.

## Non-obvious facts
- Bus B is synced by routing I2S0's SCK/WS output signals (read back from `GPIO.func_out_sel_cfg[5/6]`) to
  GPIO8/7 and running I2S1 as an RX slave on those pins. Start order: stop both, start slave, start master.
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

- Servos 2026-11-09: FS90MR continuous rotation, pan GPIO12 / tilt GPIO13 (13 assumed). Velocity-mode controller
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

## Open
- jumps.py recordings: (a) source still on a stand at 3-4 known positions (ground truth), (b) slow movement within
  ~25 deg of the axis (gimbal regime).
- servotest.py --write on the gimbal, reflash, then E1 (auto) with a tone.
- Circle test in step mode, twice in the same place, once moved; --compare to classify the 6 deg wobble.
- Tap test with `host/scope.py`; clap tests with `gcc.py` from known directions.
- Real 7.5 kHz tone test with `view.py` / `livecheck.py`.
