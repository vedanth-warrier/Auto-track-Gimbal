# CLAUDE.md

## Project
- Auto-track Gimbal (DESN2000 project, UNSW).
- Current work: bring-up of a 4-microphone I2S array on an ESP32-S3-Zero, streamed to a Mac for live plotting.
- Eventual goal: FFT / analysis on the audio. Not started. Prototype algorithms on the Mac first.

## Hardware
- Board: Waveshare ESP32-S3-Zero (native USB-C, onboard WS2812 RGB LED on GPIO21).
- Mics: 4x I2S MEMS (assumed INMP441-style; confirm the exact part before tuning bit shifts).
- Dev machine: MacBook Pro (macOS). No Windows steps.

### Mic wiring
- SCK: GPIO5 (all mics)
- WS: GPIO6 (all mics)
- Bus A, SD on GPIO3: mic 1 (L/R = 3V3, right), mic 2 (L/R = GND, left)
- Bus B, SD on GPIO4: mic 3 (L/R = 3V3, right), mic 4 (L/R = GND, left)
- Stereo frame order per bus: column 0 = left, column 1 = right.
- Pins to avoid: GPIO0, 19, 20 (USB), 21 (LED), 45, 46. GPIO3 is a strapping pin and is in use; move it if boot is erratic.

## Toolchain
- MicroPython flashed with esptool. Not PlatformIO, not Arduino.
- Firmware: plain `ESP32_GENERIC_S3` build. Not `SPIRAM_OCT`. Flash offset `0x0`.
- `mpremote` for file copy, REPL and resets. Installed via pipx or uv tool.
- Python on the Mac runs through `uv` (project `.venv`). Run scripts with `uv run`, never bare `python`.
- Mac-side deps: pyserial, numpy, matplotlib, listed in `requirements.txt`. Install with `uv pip install -r requirements.txt`.

## Layout
- `src/mic.py`: board script. Reads both I2S buses and writes `SYNC` + bus A block + bus B block to USB serial. Deployed as `main.py`.
- `src/plot.py`: Mac script. Opens the serial port with pyserial, resyncs on `SYNC`, plots 4 scrolling waveforms.
- `firmware/`: MicroPython `.bin` files. Gitignored; download link in `README.md`.

## Workflow (important)
- DO NOT run `mpremote run src/mic.py | uv run src/plot.py`. `plot.py` reads the serial port directly, so `mpremote` and `plot.py` fight over the port, and the unread pipe blocks `mpremote`. This caused the slow, garbled plot.
- Correct flow:
  - `mpremote cp src/mic.py :main.py + reset`
  - wait about 2 s for the port to re-enumerate
  - `uv run src/plot.py`
- One program holds the serial port at a time. Close the plot before using `mpremote`.
- To get the REPL back: `mpremote fs rm main.py`, then `mpremote reset`.
- `PORT` in `plot.py` is `/dev/cu.usbmodem101`. Check with `ls /dev/cu.usbmodem*` after a reset.

## Known issues and constraints
- Two MicroPython I2S peripherals sharing SCK/WS is unverified. Bus B may give garbage or shifted data. Test bus A alone first.
- No guaranteed time alignment between bus A and bus B. Not acceptable for time-difference-of-arrival or beamforming. That needs ESP-IDF (I2S0 master, I2S1 slave on shared SCK/WS).
- MicroPython is too slow for real-time on-board FFT. Plan to move to ESP-IDF + ESP-DSP if the FFT must run on the board.
- Data layout: 24-bit samples left-justified in 32-bit words, so `>> 8` on the Mac. SPH0645-type mics may need 7 or 9.
- USB stream is about 256 KB/s at 16 kHz, 4 channels. Drop to 8 kHz if there are gaps.

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
- Confirm bus A alone streams cleanly (tap each mic, check channel order).
- Confirm bus B behaviour with the shared clocks.
- Measure USB throughput (expect about 250 KB/s).
- Decide MicroPython vs ESP-IDF once the FFT size, sample rate and update rate are known.
