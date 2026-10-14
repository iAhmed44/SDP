# SDP — Sensing Subsystem

Sensing software for an AI-enabled navigation system supporting visually impaired pilgrims.

## Hardware

- Raspberry Pi 5
- EMEET C960 webcam
- RPLIDAR C1
- Five directional vibration motors

## Features

- YOLO object detection and pose estimation
- Camera and LiDAR obstacle fusion
- Colored and numbered object labels
- Initial haptic warnings followed by a 25% reminder
- Two departure pulses when an obstacle clears
- Diagnostic display and run logs

## Run on the Raspberry Pi

From the project folder, using the working Python virtual environment:

```bash
./venv/bin/python sensing_update/main_sensing.py --version
QT_QPA_PLATFORM=xcb ./venv/bin/python -u sensing_update/main_sensing.py --models cv --motors
```

Add `--no-display` to run without diagnostic windows.
Omit `--motors` to test without physical vibration.

## Files

- `sensing_update/`: application, configuration, tests and documentation
- `cv/`: YOLO model files; the active application CV module is in `sensing_update/cv/`
- `sensing_update/requirements.txt`: Python dependencies

The virtual environment and run logs are excluded from Git.
Version 8.0 is a prototype; mounted hardware validation is ongoing.
