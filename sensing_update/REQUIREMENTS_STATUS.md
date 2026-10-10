# Report requirements and validation status

Source: EE499-S26_Report-2_Team-09V6.pdf, section 1.4 (printed page 12). This current report specifies 3000 SAR, 2 kg and five hours; older conversation targets differ. No report text has been modified.

| Requirement | Sensing implementation / remaining validation |
| --- | --- |
| Hazard response 0.5 <= t < 1 second | Independent control and haptic watchdog; logs expose source delay, freshness and control gaps. Physical obstacle-to-vibration timing needs measurement. Do not add artificial delay to make alerts slower. |
| Nonvisual hazard feedback | Five directional motor commands are implemented. The user-requested maximum 1.5-second initial warning, 25% reminder and confirmed-departure/three-second healthy-absence rearm are distinct from response latency. Actual vibration and identity continuity require mounted tests. |
| Real-time navigation with <2 m accuracy | NavBridge offers expiring static snapshots. Route planning and localization are separate remaining subsystem work; this package does not implement or verify them. |
| Hands-free setup and English/Arabic/Urdu/Indonesian | Voice and language functions belong to the remaining Voice subsystem; diagnostic windows are retained as requested. |
| Navigation efficiency improvement 20–30% | Requires comparative trials with the full system. No sensing log can establish this. |
| At least five hours battery life | Requires full-load battery testing with Navigation and Voice active. |
| Cost <=3000 SAR, weight <=2 kg | Requires the final bill of materials and weighing the full device. |
| EE499 two-semester timeline | Project scheduling requirement; not established by software tests. |

## Mounted acceptance test

1. Confirm `--version` returns Sensing 8.0 and the new log header records it.
2. Run with the windows visible; check class colors and numbered display names, camera/LiDAR bearing alignment, and fusion matches.
3. Enable motors and repeat seated back-and-forth movement for at least ten seconds. The same continuously tracked obstacle should get at most one initial burst, up to 1.5 seconds, followed by a 25% duty reminder. Check encounter assignments and merge events when IDs change.
4. Introduce a distinct obstacle and verify its own warning. Confirm observed departure for at least 0.5 seconds, or absence for at least three seconds of healthy LiDAR observation; a return then gets a fresh initial warning. Check the double-pulse departure cue and that sensor failure cancels it.
5. Investigate 10–30 cm returns against the actual mounting/body position before excluding them. No blanket close-return filter has been added.
6. Stop with Ctrl+C and verify motors physically stop; check the shutdown zero-command event and logging overload/error counters.
7. Measure full response latency, supply/heat and battery life with Navigation and Voice running. Confirm GPIO17 ownership and the audio configuration before installing the planned ReSpeaker hardware (see HARDWARE_REVIEW.md).

Inference freshness does not establish detection accuracy. Record matching video or annotated observations to assess missed/false detections. Pose skeletons are available; reliable future human-intent prediction is still unimplemented.

Revision 7 uses stronger single-motor departure cues. Run the standalone departure test to evaluate perceptibility; nearby hazards retain priority on shared motors.
