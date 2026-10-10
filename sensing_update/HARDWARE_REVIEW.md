# Hardware review before Navigation + Voice integration

Reviewed: the supplied EE499-S26 Report-2 Team-09 V6, particularly printed pages 38-54 and 56-58, the uploaded sensing sources, and the official references below. This is a design/source review, not an electrical inspection or load measurement of your assembled hardware.

## The report and the current prototype differ

The report baseline uses Camera Module 3 (CSI), VL53L0X (I2C), DRV2605L-controlled haptics, ReSpeaker 2-Mic HAT, NEO-M8N GNSS, MPU6050 IMU, MAX98357A amplifier, a 4-ohm/3-W speaker and a 20,000 mAh 5 V/3 A bank. Your working sensing prototype uses a USB EMEET C960, USB RPLIDAR C1, and five separately controlled 5 V vibration modules. The old power and interface tables cannot be treated as measurements of the new assembly.

## Confirmed or material integration issues

| Issue | Why it matters | Action before integration |
| --- | --- | --- |
| GPIO17 ownership conflict | The current far-left motor uses BCM17. The ReSpeaker user button is also wired to BCM17. Configuring the same physical line as both PWM output and button input is incompatible and may create output contention. | Do not install the HAT with that shared motor connection. Move that motor to a verified free GPIO and update `haptics.pins`. A candidate is BCM5/physical29, subject to checking the exact HAT revision and every remaining connection. This package keeps today's working pin mapping until you rewire. |
| Power-bank output and USB budget | Pi5 expects suitable 5 V power; with no detected 5 V/5 A capability it limits total downstream USB current to 600 mA instead of 1.6 A. The webcam and C1 now share that USB budget. | Verify the bank's actual 5 V output/PD profile, cable rating and combined measured peak demand. A high total wattage advertised at another voltage does not establish 5 V/5 A support. Do not simply override the USB limit to compensate for an inadequate supply. |
| Shared digital audio configuration | ReSpeaker includes a codec/audio interface; MAX98357A uses I2S too. Independent sound-card overlays cannot be assumed to coexist on the same controller. | Identify HAT v1/v2 and configure/test one coherent capture+playback path. Using the HAT's supported output may avoid a second amplifier path if its speaker specifications fit. Sharing I2S is possible with the right hardware/driver arrangement, not automatic. |
| ReSpeaker revision/OS support | The report lists WM8960, corresponding to v1. Current v2 documentation uses a different codec and warns against the old driver-install method on newer Pi OS. Legacy `RPi.GPIO` button examples do not work on Pi5. | Confirm the physical revision and kernel first. Back up the boot/config files before following the matching official driver procedure. Use Pi5-compatible GPIO access. A 40-pin connector alone does not establish driver compatibility. |
| GPIO voltage and motor power | Pi signals are 3.3 V; the motor modules are 5 V powered loads. A breakout's supply tolerance does not establish its signal/pull-up voltage. | Keep PWM as a control signal, never a motor supply. Use the module's driver, common ground and a correctly rated power distribution path. Check GNSS UART and IMU/I2C pull-ups for 3.3 V compatibility. Do not backfeed independent 5 V supplies into each other. |
| Audio output power | MAX98357A can provide 3.2 W into 4 ohms at 5 V, close to/above the report's 3 W speaker rating. Its speaker output is differential. | Connect the speaker across the amplifier's two speaker terminals, not one terminal to ground. Control gain/volume and check speaker rating, clipping, current draw and heat. Do not connect two amplifiers' outputs together. |

## Processing, power and wearable operation

OD and pose are scheduled rather than run consecutively on every frame. Sensing, navigation, ASR and TTS should remain separate workers with bounded queues. Speech recognition can saturate CPU despite modest microphone electrical power; limit its active command window and avoid blocking hazard control on route planning or speech playback. The report already limits voice interaction mainly to setup and termination, which can reduce continuous ASR load. Pause or otherwise handle microphone pickup of the device's own speech so TTS does not become an unintended command source.

Pi thermal throttling slows inference under sustained load. Verify cooling airflow in the actual bag/enclosure and keep hot electronics away from skin. Measure temperature, throttling, inference age and power while OD + pose + Navigation + Voice run together; a short sensing-only desk test does not establish wearable performance. Fan, rotating LiDAR and motor vibration can also contaminate microphone recordings; test the physical microphone placement while all outputs run.

The report's 9.8 W / 6.8 h estimate was based on different sensing hardware and assumed conversion efficiency. Recalculate with measured bank-output power, including audio peaks, motor bursts, LiDAR and cooling. Avoid counting speaker load twice if amplifier input power already includes that load. Under the report's assumed 66.6 Wh usable energy, a five-hour target allows about **13.3 W average for the whole assembly**. That arithmetic does not verify the bank's usable capacity or runtime.

Secure cables/connectors and enclose exposed breadboard wiring before wearable tests. Shared high-current return paths can inject noise into GNSS/audio; use a deliberate ground/power layout rather than relying on a lightly rated breadboard as final power distribution. Use the C1's enclosed optical/mechanical assembly as supplied. Keep its scanning plane clear of the mount, head and loose cables; software cannot infer that self-returns are irrelevant without a validated mounting mask.

## Navigation-specific limits

The report's printed page 43 states a 50 Hz NEO-M8N position update rate. The manufacturer specifies **5 Hz with GPS+GLONASS and up to 10 Hz in the listed single-constellation modes**; the report's 50 Hz should be corrected. UART polling at 50 Hz cannot create new 50 Hz position fixes.

Manufacturer horizontal position accuracy is metre-scale under specified conditions, not a 0.5 m avoidance guarantee. Buildings, coverings and dense surroundings can degrade GNSS. Use it for route-scale localization; keep nearby collision warnings in the camera/LiDAR pipeline. Require valid, timestamped fixes and an explicit degraded-localization state.

MPU6050 provides accelerometer+gyroscope data, not a magnetic compass. Its six-axis measurements cannot alone establish drift-free absolute yaw. Head orientation can differ from travel direction, so rotating body-frame obstacles into a navigation map requires a defined sensor-to-body transform and a trustworthy heading estimate. Do not insert LiDAR-local x/y values directly into geographic map coordinates. Mounting calibration, pose timing and IMU synchronization are still outstanding integration work.

The optional NAV_STATIC bridge defaults to disabled and can be enabled with `--nav`. It emits complete snapshots with timestamps/TTL when explicitly enabled; the receiving Navigation subsystem must respect validity and expiry. Revision 3 isolates bridge errors in its own worker and supplies a TTL-aware receiver example. This does not reserve CPU time or isolate a shared power/OS failure. Compact logs reduce disk volume; sustained speech/navigation load and checkpoint writes still require a measured resource budget. Urgent hazards should preempt route speech, and route haptics need distinguishable patterns/ownership so they do not silently overwrite obstacle bursts. One shared output arbiter is preferable to multiple processes independently controlling the same GPIO/audio device.

## Sources

- Project: supplied EE499-S26_Report-2_Team-09V6.pdf, printed pages 38-54 and 56-58.
- Raspberry Pi hardware/power, GPIO and thermal documentation: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
- Raspberry Pi USB Power Delivery application note: https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf
- Seeed ReSpeaker overview, including GPIO17 button and WM8960: https://wiki.seeedstudio.com/ReSpeaker_2_Mics_Pi_HAT/
- Seeed revision-specific v2/Pi5 guidance: https://wiki.seeedstudio.com/respeaker_2_mics_pi_hat_raspberry_v2/
- MAX98357A manufacturer datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX98357A-MAX98357B.pdf
- u-blox NEO-M8 FW3 manufacturer datasheet: https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf
- Ultralytics pose outputs: https://docs.ultralytics.com/tasks/pose/

This update does not install audio drivers, modify boot overlays, implement GNSS/IMU navigation, or change physical wiring. Those changes require the exact remaining hardware revisions and staged integration tests.
