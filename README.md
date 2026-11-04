# Auto-track Gimbal

DESN2000 project (UNSW). A gimbal that tracks a 7 kHz tone, using 4 shrouded I2S MEMS mics on a Waveshare ESP32-S3-Zero. The board measures each mic's level at the tone and computes a direction vector; Mac scripts display it.

## Layout
- `src/mic.cpp`: board program (32 kHz, 7 kHz single-tone tracker). Streams raw 16-bit audio plus each mic's tone level and the direction vector.
- `src/servo.cpp`: board program. Servo test on GPIO12.
- `host/stream.py`: shared serial reader used by all Mac scripts (finds the port automatically).
- `host/vector.py`: tone level per mic and the direction vector.
- `host/calibrate.py`: per-mic gain calibration at the tone frequency.
- `host/plot.py`: live spectrum (0 to 16 kHz) for all 4 mics.
- `host/peak.py`: level of the loudest frequency for each mic over time.
- `host/scope.py`: 4 scrolling waveforms, for tap tests.
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
uv run host/vector.py
```
Only one program can hold the serial port at a time. Close plots before uploading.
`pio run -e mic_test -t upload` flashes a bench build that replaces the mics with a synthetic 7 kHz tone.
