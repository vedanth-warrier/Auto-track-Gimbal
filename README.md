# Auto-track Gimbal

DESN2000 project (UNSW). 4-microphone I2S array on a Waveshare ESP32-S3-Zero, streamed over USB to a Mac for live plotting.

## Layout
- `src/mic.cpp`: board program. Reads both I2S buses and streams `SYNC` + bus A + bus B over USB serial.
- `src/servo.cpp`: board program. Servo test on GPIO12.
- `host/plot.py`: Mac script. Live FFT spectrum (dBFS vs frequency) for all 4 mics.
- `host/scope.py`: Mac script. 4 scrolling waveforms, for tap tests.
- `legacy/micropython/`: the original MicroPython versions, kept for reference.

## Setup
1. Install PlatformIO (VS Code extension, or `pipx install platformio`).
2. Mac dependencies:
   ```
   uv venv
   uv pip install -r requirements.txt
   ```

## Run
```
pio run -e mic -t upload
uv run host/plot.py
```
Only one program can hold the serial port at a time. Close the plot before uploading.
