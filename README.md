# Auto-track Gimbal

DESN2000 project (UNSW). 4-microphone I2S array on a Waveshare ESP32-S3-Zero, streamed over USB to a Mac for live plotting.

## Layout
- `src/mic.py`: board script. Reads both I2S buses and streams `SYNC` + bus A + bus B over USB serial. Deploy as `main.py`.
- `src/plot.py`: Mac script. Plots 4 scrolling waveforms from the serial stream.
- `src/servo.py`: standalone servo test (GPIO12).

## Setup
1. Download MicroPython for `ESP32_GENERIC_S3` (plain build, not SPIRAM) from https://micropython.org/download/ESP32_GENERIC_S3/ into `firmware/`.
2. Flash it at offset `0x0` with esptool.
3. Mac dependencies:
   ```
   uv venv
   uv pip install -r requirements.txt
   ```

## Run
```
mpremote cp src/mic.py :main.py + reset
# wait ~2 s for the port to re-enumerate
uv run src/plot.py
```
Only one program can hold the serial port at a time. Close the plot before using `mpremote`.
