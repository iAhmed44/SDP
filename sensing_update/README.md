# Wearable Sensing — Start Here

This program uses the webcam and LiDAR to find nearby obstacles and give directional vibration feedback. It keeps object detection, pose skeletons, class colors and numbered display labels. The diagnostic windows remain enabled.

## Install and run

Stop the running program, download sensing_update_v8.zip to Downloads, then run:

```bash
cd /home/aarteam09/Sensing_SubS
unzip -o "$HOME/Downloads/sensing_update_v8.zip" -d .
./venv/bin/python sensing_update/main_sensing.py --version
```

The version must show **Sensing 8.0**. Your existing configuration, alignment, model weights, venv and logs are preserved. The new haptic defaults are merged into your existing settings.

First check the display with motors disabled:

```bash
QT_QPA_PLATFORM=xcb ./venv/bin/python -u sensing_update/main_sensing.py --models cv
```

Then enable the physical motors:

```bash
QT_QPA_PLATFORM=xcb ./venv/bin/python -u sensing_update/main_sensing.py --models cv --motors
```

Press Q, Esc or Ctrl+C to stop. Run only one program that uses the camera or LiDAR at a time.

## New vibration pattern

| Situation | Feedback |
| --- | --- |
| A newly warned obstacle | Directional proximity warning for up to 1.5 seconds |
| It remains hazardous | Continuous reminder with a maximum 25% PWM duty on the directional motors |
| It leaves the warning zone | Two 300 ms pulses at 100% duty, with a 250 ms silent gap on one directional motor |
| LiDAR fails or becomes stale | Motor output stops; sensor failure does not trigger departure pulses |

Departure requires 0.5 seconds of observed geometric clearance or three seconds of absence during healthy LiDAR operation. Losing only a camera match is insufficient. The cue describes one obstacle leaving the zone; it does not guarantee the whole path is clear. Warnings on other motors continue while the departure cue plays. A hazard using the cue motor defers its start for up to three seconds. If that motor becomes needed during a cue, the cue stops. A returning hazard from the same encounter cancels its pending cue.

A return after confirmed departure starts a fresh initial warning of up to 1.5 seconds, followed by the 25% reminder. Brief movement within the zone, a short dropout, a class-label change or a matched cluster-ID change alone does not restart the strong warning. Three seconds of healthy observed absence also rearms it. Existing independent tracks keep separate warning histories when their surfaces overlap. Display names such as person1 are only labels, not remembered people. Restarting the program clears encounter memory.

PWM duty does not directly measure vibration strength. Check that your modules actually vibrate at 25% and that the double pulse is distinguishable. Reminder output bypasses the normal minimum-duty adjustment so its peak command is exactly 25% with the default configuration.

## Feel the departure pattern by itself

Stop the full sensing program first. Test the center motor with:

```bash
./venv/bin/python -u sensing_update/test_departure_pattern.py --motors --index 2
```

This sends only the double pulse and clears the motor afterward. It starts no camera or LiDAR. Indices: 0 far left, 1 left, 2 center, 3 right, 4 far right. Omitting `--motors` performs a dry run. The output should report 300 ms / 100%, 250 ms / 0%, 300 ms / 100% with the new defaults; existing custom config overrides still apply.

## Camera labels

Each class has a consistent color. Objects in the same class are numbered left-to-right in each result: person1, person2, chair1. These are display-order numbers and may change when objects cross. LiDAR tracks control warning identity.

## Camera-only test

Stop sensing first, then run:

```bash
QT_QPA_PLATFORM=xcb ./venv/bin/python -u sensing_update/test_cv_only.py --models cv
```

This starts camera detection and pose with no LiDAR or motor output.

## Navigation connection test

Start sensing with `--nav`, then run this in a second terminal from Sensing_SubS:

```bash
./venv/bin/python -u sensing_update/test_nav_bridge.py
```

NavBridge transmits expiring static-obstacle snapshots. Route planning remains separate.

## Logs and next test

Each run creates a compressed file in **sensing_update/Sensing_LOG/**. The version, CV timing, LiDAR health, encounter events and motor phases are grouped into ten-second sections. The final group is saved when stopping. Wait for `[LOG] Finished:` before uploading the latest .log.gz.

Test an obstacle for more than ten seconds: initial warning, then the 25% reminder. Move it outside the warning zone and check the double pulse. Confirm a sensor disconnect stops motors without giving that departure cue. Upload the resulting log for review.

See TECHNICAL_GUIDE.md for implementation details, HARDWARE_REVIEW.md for pending hardware integration, and REQUIREMENTS_STATUS.md for the current report requirements and validation limits.
