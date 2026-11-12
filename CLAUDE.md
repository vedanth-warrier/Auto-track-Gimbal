# CLAUDE.md

## Project
- Auto-track Gimbal (DESN2000 project, UNSW).
- Target: the gimbal tracks a 7 kHz tone (beacon), not voices.
- Current work: single-tone tracker on an ESP32-S3-Zero with a 4-mic I2S array at 32 kHz. Board computes each mic's level at the tone and a direction vector; Mac scripts plot it.
- Method: shrouded mics, each tilted 30 deg from vertical, compared by tone amplitude for 2-axis direction. Not TDOA.
- Next: calibrate gains at 7 kHz, measure 7 kHz directivity, then the servo loop.

## Hardware
- Board: Waveshare ESP32-S3-Zero (native USB-C, onboard WS2812 RGB LED on GPIO21).
- Mics: 4x I2S MEMS (assumed INMP441-style; confirm the exact part before tuning bit shifts).
- Dev machine: MacBook Pro (macOS). No Windows steps.

### Mic wiring
- Bus A (I2S0): SCK GPIO5, WS GPIO6, SD GPIO3. Mic 1 (L/R = 3V3, right), mic 2 (L/R = GND, left)
- Bus B (I2S1): SCK GPIO8, WS GPIO12 (moved from GPIO7 2026-11-12: GPIO7 joint failed continuity), SD GPIO4. Mic 3 (L/R = 3V3, right), mic 4 (L/R = GND, left)
- Stereo frame order per bus: column 0 = left, column 1 = right. Confirmed under Arduino by tap test, 2026-11-04.
- Mic facing directions (`AZ_DEG`, degrees around the vertical, anticlockwise from above): mic 1 = 90, mic 2 = 180, mic 3 = 0, mic 4 = 270. Must match in `mic.cpp` and `vector.py`.
- SCK runs at 2.048 MHz at 32 kHz (64 clocks per frame). Jumper wiring into the shroud.
- Pins to avoid: GPIO0, 19, 20 (USB), 21 (LED), 45, 46. GPIO3 is a strapping pin and is in use; move it if boot is erratic.

### Servos (gimbal)
- 2x FS90MR continuous rotation (no position feedback), 5 V from the laptop. Pan GPIO10 (about the vertical, ±90 from centre), elevation GPIO11 (0 = plate facing the horizon = bottom of travel, 90 = facing up). Quarter sphere.
- Plate facing the horizon: mic 2 up, mic 3 down, mic 4 / mic 1 at the sides. Pan FORWARD (pulse > 1500) turns towards mic 4; elevation FORWARD towards mic 2.
- No hard stops yet: the servos free-spin and pull the wires past the ends.
- Gearing: elevation via a 5:1 worm gear with consistent slack on direction changes (zeroed with the slack taken up for moving up); pan geared lower than 5:1. Usable pulse offsets 300-800 us.

### Hardware handling
- Unplug the ESP32's USB before plugging, unplugging or swapping mics or wires. SCK/WS run whenever the firmware runs, so a mic that loses VDD/GND before its clock pins gets back-powered through them (latch-up or damage risk).
- Close any script holding the serial port first.
- Touch something grounded before handling mic modules (ESD).
- 2026-10-15: two mic modules stopped outputting after being unplugged with the board powered. Cause not confirmed (bad joint, latch-up, ESD or damage).

## Toolchain
- Arduino framework (arduino-esp32 2.0.17) via PlatformIO, platform `espressif32@7.1.3`. Switched from MicroPython 2026-10-15.
- `pio` lives at `~/.platformio/penv/bin/pio` (installed by the VS Code extension), not on PATH.
- USB: `ARDUINO_USB_MODE=1` (USB-Serial-JTAG). Uploads auto-reset over the same port.
- Python on the Mac runs through `uv` (project `.venv`). Run scripts with `uv run`, never bare `python`.
- Mac-side deps: pyserial, numpy, matplotlib, listed in `requirements.txt`. Install with `uv pip install -r requirements.txt`.

## Layout
- `platformio.ini` envs:
  - `mic`: the tracker.
  - `mic_test`: same code, but the mic input is replaced by a synthetic 7 kHz sine (mic 1 at 2x the others). Every output is known exactly; use it to check the pipeline after code changes.
  - `mic_timing`: `mic_test` plus per-stage microseconds in `amp[0..2]` (append, spectra, whole loop body).
  - `servo`: servo oscillation test (`src/servo.cpp`, GPIO10).
  - `servo_ctl`: servo controlled from the Mac (`src/servo_ctl.cpp` + `host/servo_gui.py`).
- `src/mic.cpp`: 32 kHz, packets of 256 samples per mic (8 ms, 125/s).
  - Raw audio streamed as int16 (top 16 bits of the 24-bit sample).
  - 2048-point Hann FFT (15.6 Hz bins; 7 kHz = bin 448). Mics FFT'd in pairs (real + imaginary) and separated, so 2 FFTs per packet; only bins within +/- `FLOOR_HZ` of the tone are kept.
  - Tone: strongest bin of the summed spectra within `TONE_HZ` +/- `TONE_TOL_HZ`, at least `DETECT_DB` (12) above the median within +/- 1 kHz, for `MIN_RUN` (6) packets in a row within +/- 2 bins. Otherwise the DIRV packet is zeroed.
  - Per-mic amplitude = power in the peak bin +/- 1 (all of an on-bin tone's energy with Hann), gain corrected, as RMS. Vector = amplitude-weighted mean of the mic facing directions.
- `src/servo.cpp`: oscillation test for the FS90MR on GPIO10 (GPIO12 is now bus B WS: never drive it), from every boot: left for `A_MS`, pause, right for `A_MS`, repeat. Raise `A_MS` until each swing is 90 deg. LED red = left, blue = right. Pins verified 2026-11-12 by pad readback (1198/1799 us, 20.0 ms period on both). No serial output: see Known issues.
- `src/servo_ctl.cpp`: two FS90MR (servo 1 GPIO10, servo 2 GPIO11) driven over serial: `F<n> <us>` forward, `R<n> <us>` reverse (n = 1 or 2, offset from 1500, 0-800), `S` stops both; replies `OK <n> <pulse>`. One servo runs at a time. Stops if no command for 300 ms. LED green = forward, blue = reverse.
- `host/stream.py`: shared serial reader. Finds the port (`/dev/cu.usbmodem*`), opens it with DTR/RTS low, parses packets, calls `on_audio` / `on_dir`. The packet format lives here and in `mic.cpp` only.
- Packet: `SYNC` + bus A (1024 B) + bus B (1024 B, each 256 stereo frames int16 [left, right]) + `DIRV` + 8 float32 (4 amplitudes, x, y, tone Hz, SNR dB). 2088 bytes, little-endian.
- `host/vector.py`: tone level bars per mic, raw and smoothed vector (`TAU`), `SCALE` display gain, angle in the title. `WEIGHTS` (mic 1..4) multiply the board's per-mic levels and the vector is recomputed on the Mac, for trial-and-error tuning without reflashing; all 1 = the board's own vector.
- `host/servo_gui.py`: servo control window for `servo_ctl`: SERVO 1 / SERVO 2 select buttons (switching stops both), hold FORWARD / REVERSE, speed slider + text box (us from neutral). `FORWARD_SIGN` per servo. Resends every 0.1 s while held; shows the board's reply.
- `host/plot.py`: live spectrum per mic (4096-point, dBFS, 0 to 16 kHz).
- `host/peak.py`: level of the loudest bin per mic over 10 s, plus its frequency.
- `host/scope.py`: 4 scrolling waveforms (min/max envelope). Use for tap tests.
- `host/calibrate.py`: gain calibration at `TONE_HZ`. Tone above the array centre; 4 rounds, array rotated 90 deg between; prints the `GAIN` line for `mic.cpp`.
- `legacy/micropython/`: old MicroPython scripts, for reference only.

## Workflow (important)
- Flash: `~/.platformio/penv/bin/pio run -e mic -t upload` (or `-e mic_test`, `-e servo`).
- The VS Code PlatformIO Upload button flashes `default_envs` in the root `platformio.ini` (set to `servo_ctl` 2026-11-12; was `mic`). It never flashes `tdoa/`: use the terminal for that.
- Then: `uv run host/vector.py` (or another host script).
- One program holds the serial port at a time. Close plots before uploading or running another script.
- Upload sometimes fails with "chip stopped responding" while the board is streaming. Run it again.
- If an upload can't connect at all: hold BOOT, tap RESET, release BOOT, upload again, then tap RESET.

## Known issues and constraints
- Timing (measured, `mic_timing`): append 62 us, 2 paired FFTs + split 3043 us, whole loop body 3205 us (p99 3208) of the 8000 us budget. Unpaired 4 FFTs took 6608 us, too slow.
- Stream measured 124.96 packets/s, 260.9 KB/s. 16-bit packing chosen to stay at the proven ~256 KB/s; 32-bit at 32 kHz would need 517 KB/s (untested). 16-bit quantisation floor (about -98 dBFS) is below the INMP441 noise floor (about -87 dBFS, datasheet figure from memory).
- `mic_test` (synthetic 7 kHz, 0.01 FS, mic 1 at 2x): tone 7000.00 Hz, amplitudes and vector match expected to 4-5 significant figures; raw stream amplitudes exact.
- False tone outputs on room sound: 0 to 19 per 7500 packets (0 to 0.25%) with `DETECT_DB` 12 and `MIN_RUN` 6. Without persistence, 0.2 to 1% (single-packet hits up to 3 in a row, SNR 12-13 dB).
- At 32 kHz the stream shows 4-14 isolated single-sample spikes per second per mic, about 0.03 FS each, rarely on two mics at once: likely bit errors from the faster SCK over jumpers. Effect on the 7 kHz noise floor: 0.00 to 0.05 dB. Shorten SCK/WS jumpers if it grows.
- `GAIN` in `mic.cpp` is now {1.312, 0.927, 0.727, 1.131} (set by user 2026-11-04, presumably from calibrate.py at 7 kHz). `GAIN` applies only to the DIRV output; plot/scope/peak.py show uncorrected levels.
- Gain calibration: single-run offsets moved by up to 2.3 dB when the array was rotated, so average over rotations (calibrate.py does 4).
- Mic 4 extra noise (up to +24 dB at 6-8 kHz) was a wiring fault at position 4; fixed 2026-11-04.
- 440 Hz sawtooth sweeps (16 kHz, harmonic method, 2026-11-04): per-mic level change with direction +/-0.6 to 2.2 dB; at 7040 Hz +/-2.2 dB with all 4 mics agreeing within 18 deg on a common offset of -131 deg from `AZ_DEG`. To be re-measured at 7 kHz with the tone tracker.
- ESP-DSP optimised FFT needs 16-byte aligned buffers (`__attribute__((aligned(16)))`). Unaligned gave about +24 dBFS on every mic.
- Arduino defines `BIN` as a macro (Print.h). Don't use it as a name.
- USB serial (HWCDC, core 2.0.17): never call `Serial.setTxTimeoutMs(0)`. Its retry counter starts at the timeout and is decremented before the zero check, so 0 wraps and a write blocks ~49 days whenever the Mac is plugged in but no program reads the port (ring buffer 256 B fills). Hit 2026-11-12 in servo.cpp: servo kept turning at the last pulse. Default (100 ms) times out instead.
- Onboard WS2812 (GPIO21) has red and green swapped vs `neopixelWrite(pin, r, g, b)`.
- Bus A and bus B have separate SCK/WS: arbitrary phase offset between buses at each boot (unverified). Irrelevant for amplitude comparison.
- MicroPython (v1.29 generic S3) capped stdout at 11.4 KB/s, measured on the board. This is why we switched.
- DTR/RTS on the USB port drive the chip's reset/boot lines. `stream.py` opens with them low; any other script must too.
- After manually entering download mode, the software reset leaves the chip in the bootloader. Tap RESET (not BOOT) after flashing.

## How to work with me
- Dot points over paragraphs. Short and direct.
- Brutal honesty. If something is weak or won't work, say so. No softening.
- No validating openers ("good catch", "fair point"). Lead with the substance.
- When changing existing files, give only the exact lines to change. Do not reprint whole files.
- Done items get deleted from any tracked list, never struck through.
- Any figure or claim I might be held to must be defensible. Flag anything that isn't.
- Say plainly when something is unverified (e.g. firmware behaviour I can't confirm).
- Writing rules: Australian/British spelling (colour, optimise, modelling). No em-dashes. Avoid "genuinely", "honestly", "straightforward". Don't editorialise about difficulty.
- I often dictate. Garbled technical terms are transcription errors, not knowledge gaps. Ask if a term looks wrong.

## Open actionables
- Real-tone check in `vector.py` (detection, SNR, vector direction per mic).
- 8-direction directivity sweep at 7 kHz.
- Gimbal (`tdoa/src/gimbal.cpp`): `tdoa/host/gimbal_cal.py --write` (easy angles: elevation 0/90, pan -90/+90), reflash, real-tone test with `gimbal_view.py`. Then hard stops + re-homing against them.
