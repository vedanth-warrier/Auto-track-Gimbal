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

## Toolchain
- Arduino framework (arduino-esp32 2.0.17) via PlatformIO, platform `espressif32@7.1.3`. Switched from MicroPython 2026-10-15.
- `pio` lives at `~/.platformio/penv/bin/pio` (installed by the VS Code extension), not on PATH.
- USB: `ARDUINO_USB_MODE=1` (USB-Serial-JTAG). Uploads auto-reset over the same port.
- Python on the Mac runs through `uv` (project `.venv`). Run scripts with `uv run`, never bare `python`.
- Mac-side deps: pyserial, numpy, matplotlib, listed in `requirements.txt`. Install with `uv pip install -r requirements.txt`.

## Layout
- `platformio.ini`: one env per board program (`mic`, `servo`), selected with `build_src_filter`.
- `src/mic.cpp`: reads both I2S buses (legacy `driver/i2s.h`). Per packet: writes `SYNC` + bus A block + bus B block, then `DIRV` + 4 band-RMS amplitudes + vector x, y (float32 LE, 28 bytes). Amplitude = RMS in `BAND_LO..BAND_HI` from a 1024-point ESP-DSP FFT. Vector = amplitude-weighted mean of mic facing directions (`AZ_DEG`), length 0..1.
- `src/servo.cpp`: servo test on GPIO12 (LEDC PWM, 50 Hz, 600 to 2400 us).
- `host/plot.py`: Mac script. Background thread reads the serial port and resyncs on `SYNC`. Plots a live FFT spectrum per mic (2048-point, Hann window, dBFS, 0 to 8 kHz), all 4 overlaid.
- `host/peak.py`: Mac script. FFT every packet (62.5/s); picks the loudest bin above 100 Hz across all 4 mics and plots each mic's dBFS at that bin over the last 10 s, plus the bin frequency.
- `host/vector.py`: Mac script. Reads `DIRV` packets; bar chart of band level per mic (dBFS) and the direction vector with a 1 s trail. `AZ_DEG` must match `mic.cpp`.
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
- Board band RMS verified against numpy on the same audio: 0.0000 dB max difference over 624 packets.
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
- Map vector to gimbal angles (needs the directivity data); add a level gate so silence doesn't steer.
