# CLAUDE.md

## Project
- Auto-track Gimbal (DESN2000 project, UNSW).
- Current work: 4-microphone I2S array on an ESP32-S3-Zero, streamed to a Mac for live plotting. All 4 mics confirmed working (under MicroPython).
- Method: shrouded mics, each tilted 30 deg from vertical, compared by FFT band amplitude for 2-axis direction. Not TDOA.
- Eventual goal: FFT band levels on the board, driving the gimbal servos. Prototype algorithms on the Mac first.

## Hardware
- Board: Waveshare ESP32-S3-Zero (native USB-C, onboard WS2812 RGB LED on GPIO21).
- Mics: 4x I2S MEMS (assumed INMP441-style; confirm the exact part before tuning bit shifts).
- Dev machine: MacBook Pro (macOS). No Windows steps.

### Mic wiring
- Bus A (I2S0): SCK GPIO5, WS GPIO6, SD GPIO3. Mic 1 (L/R = 3V3, right), mic 2 (L/R = GND, left)
- Bus B (I2S1): SCK GPIO8, WS GPIO7, SD GPIO4. Mic 3 (L/R = 3V3, right), mic 4 (L/R = GND, left)
- Stereo frame order per bus: column 0 = left, column 1 = right.
- Pins to avoid: GPIO0, 19, 20 (USB), 21 (LED), 45, 46. GPIO3 is a strapping pin and is in use; move it if boot is erratic.

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
- `platformio.ini`: one env per board program (`mic`, `servo`), selected with `build_src_filter`.
- `src/mic.cpp`: reads both I2S buses (legacy `driver/i2s.h`). Per packet (62.5/s): writes `SYNC` + bus A block + bus B block, then `DIRV` + 4 amplitudes + x + y + f0 + harmonics used (8 float32 LE, 36 bytes). Harmonic method: 1024-point ESP-DSP FFT per mic; pitch from the loudest peak tried as harmonic 1..6, accepted if at least 60% of its first 10 harmonics clear the local noise floor + 10 dB; then each harmonic of f0 inside `BAND_LO..BAND_HI` gives a per-mic comparison, combined weighted by harmonic amplitude. Fewer than 3 in-band harmonics -> all zeros. Pitched sounds only, by design.
- `src/servo.cpp`: servo test on GPIO12 (LEDC PWM, 50 Hz, 600 to 2400 us).
- `host/plot.py`: Mac script. Background thread reads the serial port and resyncs on `SYNC`. Plots a live FFT spectrum per mic (2048-point, Hann window, dBFS, 0 to 8 kHz), all 4 overlaid.
- `host/peak.py`: Mac script. FFT every packet (62.5/s); picks the loudest bin above 100 Hz across all 4 mics and plots each mic's dBFS at that bin over the last 10 s, plus the bin frequency.
- `host/vector.py`: Mac script. Reads `DIRV` packets; bar chart of harmonic level per mic (dBFS), the direction vector with a 1 s trail, f0 and harmonic count. `AZ_DEG` must match `mic.cpp`.
- `host/scope.py`: Mac script. Same reader, plots 4 scrolling waveforms (min/max envelope). Use for tap tests.
- `legacy/micropython/`: old MicroPython scripts, for reference only.

## Workflow (important)
- Flash: `pio run -e mic -t upload` (or `-e servo`).
- Then: `uv run host/plot.py`
- One program holds the serial port at a time. Close the plot before uploading or opening the serial monitor.
- `PORT` in `plot.py` is `/dev/cu.usbmodem101`. Check with `ls /dev/cu.usbmodem*` if it changes.
- If an upload can't connect: hold BOOT, tap RESET, release BOOT, upload again, then tap RESET.

## Known issues and constraints
- Bus A and bus B have separate SCK/WS. Same clock source and rate, so no drift expected, but the phase offset is arbitrary at each boot (unverified). Irrelevant for amplitude comparison; matters only for TDOA.
- MicroPython (v1.29 generic S3) capped stdout at 11.4 KB/s, measured on the board. Likely cause: stdout duplicated to UART0 at 115200. This is why we switched.
- Stereo slot order (column 0 = left) was confirmed under MicroPython only. Re-check under Arduino.
- ESP-DSP optimised FFT needs 16-byte aligned buffers (`__attribute__((aligned(16)))`). Unaligned gave about +24 dBFS on every mic.
- Harmonic method, tested in Python on synthetic harmonic sources (150/220/440 Hz) mixed with 15 s of real room noise (band RMS about -61 dBFS): false detections on room noise alone 1.9% of packets; detection 41-82% at -40 dBFS source, 82-90% at -30, 99-100% at -20; x within 0.01 of truth at -20 dBFS. Synthetic only; not yet tested with a real sound source.
- 2026-11-04: mic 4 (new module) has a flat broadband noise floor about -41 dB per bin from 0.5 to 8 kHz: +12.5 dB above the others at 2-4 kHz, +18 at 4-6, +24 at 6-8. No bit errors or glitches, so not a data-wiring fault. Module or its VDD/GND wiring; swap test not done yet.
- Global-median threshold (first version) gave 85% false detections on room noise. Peaks must be judged against a local floor.
- Arduino defines `BIN` as a macro (Print.h). Don't use it as a name.
- Data layout: 24-bit samples left-justified in 32-bit words, so `>> 8` on the Mac. SPH0645-type mics may need 7 or 9.
- USB stream: measured 256.5 KB/s, 62.51 packets/s (16.0 kHz) under Arduino, 20 s steady state after draining the start-up backlog.
- DTR/RTS on the USB port drive the chip's reset/boot lines. Open the port with `dtr = rts = False` (as `plot.py` does) or the board can drop into the bootloader.
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
- Tap test under Arduino to re-confirm channel order.
- Directivity sweep per shrouded mic to pick the FFT band, then set `BAND_LO`/`BAND_HI` in `mic.cpp`.
- Set `AZ_DEG` in `mic.cpp` and `vector.py` to the real mic layout (placeholder: 0, 90, 180, 270).
- Per-mic gain calibration, into `GAIN` in `mic.cpp`.
- Mic 4 noise: swap its module with mic 2 (USB unplugged). Noise follows module -> replace it; stays in position -> check mic 4's VDD/GND jumpers.
- Map vector to gimbal angles (needs the directivity data); add a level gate so silence doesn't steer.
