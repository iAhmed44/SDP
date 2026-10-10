# Revision 8 validation

104 automated tests passed. Six added regressions cover confirmed departure with the same track, replacement-track continuity, brief clearance, sensor loss without departure, semantic label changes, and independently tracked overlapping objects plus a new bridging cluster. The sensor-loss regression now expects a fresh warning when departure was already confirmed before the outage. Existing split/merge replay, timing, GPIO watchdog/shutdown, CV scheduling/colors/labels, fusion, departure arbitration, and navigation-message tests remain passing. Test sensors/models/GPIO are simulated; this does not establish physical vibration or detection accuracy.

The final archive was extracted into a temporary installation: existing config.json was preserved, --version returned Sensing 8.0, and a headless simulated run produced initial/reminder phases and a clean shutdown with zero commands. Archive CRC validation passed. The isolated departure dry run commanded 300 ms at 100%, 250 ms off, then 300 ms at 100%, with successful final zero commands. The largest recorded control-cycle gap in the uploaded run was 226.9 ms; physical motor response was not measured.

## Latest uploaded run

`Sensing_LOG_20261011_005155_155003.log.gz` records version 7.0. Encounter 7 confirms departure at approximately 12.99 seconds but returns as a reminder at 16.22 seconds. Its history repeatedly absorbs other encounters. Revision 8 fixes the missing confirmed-departure rearm and incidental merging of already-known tracks. The new regression uses representative synthetic geometry at these times; compact logs do not contain enough raw scans for an exact physical-scene replay.

The run contains 530 OD and 264 pose results. Median inference times are 224.6 ms and 270.5 ms; median capture-to-result ages are 265.1 ms and 306.5 ms respectively. 793 of 794 results are marked valid; the first warm-up result takes about 2.28 seconds and is rejected as stale. These figures explain visible update lag and are not an accuracy score or end-to-end physical response measurement. Of 792 sampled state records, 787 report healthy LiDAR and 759 report healthy vision; these are sample counts, not uptime percentages.

There are 21 departure confirmations, 12 completed cue records, 8 cancellations and 1 expiry. Completion means both stages were commanded, not measured vibration. Shutdown at 203.507 seconds reports successful zero commands for all five motors, and the log has an END OF RUN marker. No traceback or dropped/overflow text was found; absence of such text does not prove lossless logging.

Revision 8 keeps the 25% continuous reminder, directional two-pulse departure settings, diagnostic windows, class colors, numbered labels, and sensor-failure output clearing. Complete project requirements involving mounted hardware, real-world accuracy, battery, weight and navigation still require physical/integration evidence as described in REQUIREMENTS_STATUS.md.

## Previous validation

# Revision 7 validation

98 tests passed, including free-bearing concurrent cues, busy-motor deferral, interruption by a same-motor hazard, the longer double pulse, and explicit incomplete reporting when the second pulse stage is missed. Revision-6 tests explicitly select the older configurable pulse durations while the revision-7 tests exercise current defaults. The standalone test completes its two-pulse dry run and reports successful zero commands.

The uploaded Sensing_LOG_20261011_004127_646268.log.gz is version 6.0. It contains ten departure confirmations and eight cancellation records. Six confirmations are immediately cancelled because another hazard is present; another cue begins at 38.940 seconds and is cancelled at 38.993 seconds, approximately 53 ms later; the final confirmation at 122.817 seconds is cancelled immediately. Both commanded pulses appear at 42.159/42.464 seconds and 78.506/78.808 seconds. These records do not establish physical vibration strength. Shutdown reports successful zero commands for all motors.

Correction: the confirmation/cancellation totals cannot be subtracted to count completed cues because confirmation and cancellation occur at different times. Two cues show both recorded pulse stages. Revision 7 removes global cancellation and increases pulse strength/duration while retaining directional hazard priority. The next log and isolated physical test must verify perceptibility.

## Historical validation

# Revision 6 validation

93 automated tests passed, including ten sustained-reminder and departure-pattern tests.

New tests cover initial-to-reminder transition, exactly 25% peak directional PWM, no repeated strong burst after ID changes or dropout, two distinct departure pulses and gap, geometric confirmation, camera-match loss, three-second healthy absence, sensor-outage cancellation, competing-hazard priority and reminder watchdog shutdown. Legacy silent-after-burst tests explicitly select that compatibility setting.

The uploaded Sensing_LOG_20261011_002222_481625.log.gz confirms revision 5.0. It contains 1074 CV results (1070 valid), median frame-to-result delay 272.9 ms and p95 359.5 ms. Detection: 716 results, median 220.2 ms and p95 293.5 ms. Pose: 358 results, median 252.3 ms and p95 317.2 ms. Maximum recorded control-loop gap: 216.3 ms. The run ends at about 281.8 seconds; its compressed size is 626125 bytes.

There are 57 first-warning events, with no track ID receiving a second first-warning event in this run. That supports improved track-history continuity; it does not prove that every physical object retains one identity through all cluster changes. Shutdown successfully records zero commands on all five motors. No CV error result is present, and final logging counters contain no overload-drop/eviction entries.

Version 6 changes output semantics: reminder output continues after the strong burst. Compare haptic_phase and duties in the next log. Test actual vibration at low duty, perceptibility of the two pulses, physical stop on sensor loss, full system response latency and battery life. PWM command percentages are not measured motor power or vibration amplitude. Reliable future human-intent prediction remains outside this update.

## Historical validation

# Revision 5 validation

83 automated tests passed. New checks verify per-class numbering, stable colors, unchanged semantic classes/caches, actual renderer output, versioned logs, immediate encounter-event logging between snapshots, and encounter event lifecycle. The inherited tests cover identity priority, split/merge history, bounded burst duration, dropouts, watchdogs, logging recovery and NavBridge isolation. Model inference is mocked and motors simulated; these tests do not establish Pi performance or physical response compliance.

## Latest uploaded run

Sensing_LOG_20261008_233100_842708.log.gz contains a complete run ending at about 100.3 seconds. Compressed size: 197,439 bytes. It records 291 detection results (median 200.1 ms, p95 230.8 ms) and 145 pose results (median 233.5 ms, p95 264.7 ms). Combined rate: 4.58 results/second; 435 of 436 results valid at publication. Detection accuracy cannot be determined without corresponding images/video.

There are 64 warning start records. Track 300 starts encounters 159 and 160 at 93.818 and 93.921 seconds, respectively. This is incompatible with the intended three-second absence rule for a continuously known track. Other repeated track starts also occur. Very close unknown returns remain frequent; the log cannot identify their physical origin.

This run has neither `frame_to_result_ms` nor `control_cycles_since_record`, both introduced in revision 4. It therefore does not validate the revision-4 identity/timing code. Revision 5 makes the installed version explicit. The shutdown record reports zero commands for all five motors successfully; physical output is not measured. The final logging counters contain no overload-drop or eviction entries.

The full point geometry and every control cycle are omitted in compact logs, so an exact replay of the failure is unavailable. No claim is made that compact replay proves physical duplicate suppression. Repeat the mounted experiment after confirming Sensing 5.0.

See REQUIREMENTS_STATUS.md for current report requirements and remaining system-level validation.

## Prior validation history

# Revision 4 validation

79 automated tests passed in this environment on 8 October 2026. Tests use simulated camera/model outputs and motor drivers; no Pi hardware measurements were performed here.

A new adversarial test verifies that a continuously known track retains suppression even after movement beyond the spatial fallback gate while another cluster occupies its old location. Other tests cover generation-qualified IDs, healthy-absence rearming, all-cycle logging aggregates, and CV timing fields.

The compact observations of track 55 from elapsed 17–24 seconds retain one burst deadline in replay. Both old and new code pass that reduced replay: compact logs omit point geometry and intervening cycles, so this is NOT an exact reproduction or proof that the physical failure is solved. Repeat the chair experiment on the Pi.

## Latest recorded CV performance

Source: Sensing_LOG_20261008_213018_345762.log.gz, approximately 74 seconds.
- 217 detection results: median 191.9 ms, p95 225.6 ms.
- 108 pose results: median 222.6 ms, p95 246.2 ms.
- 325 results total; 324 marked valid at publication. The first result takes about 2.1 seconds during startup and is invalid.
- 4.72 combined results/second from first to last publication; individual OD and pose rates are lower because they alternate.
- Logged source-to-finished delay: median 236.9 ms, p95 290.4 ms. In revision 3 this excludes the final background-motion processing; revision 4 fixes that timestamp boundary.
- 267 of 294 sampled/health-transition records report healthy vision. These samples are not uniformly spaced, so this fraction is NOT uptime.

The log confirms execution and freshness, not detection accuracy. The report section 1.4.1 specifies hazard response as 0.5 <= t < 1 second. Neither inference times nor command logs alone establish full physical response compliance. The 1.5-second burst duration is separate from response latency.

## Earlier validation (historical)

# Revision 3 validation

Development checks completed on 8 October 2026. **74 automated tests passed.** The Pi hardware and actual model inference were not available in this environment.

## Recorded warning regression

The supplied `Sensing_LOG_20261008_071700_741838.log` copy is 11,202,560 bytes and ends inside a LiDAR JSON record at elapsed 27.195 seconds. It has no final shutdown record. This may reflect an incomplete upload/copy or an interrupted log; the file alone cannot distinguish them. Analysis below uses the complete first two 10-second groups, not the incomplete tail.

In that interval, 347 matched LiDAR/haptic cycles, 77 CV results and 374 terminal records were available. The same return near +23° and 1.1 m moved through cluster IDs 5, 8 and 10 as its surface split/merged. Revision 2 created separate warning encounters and new deadlines. Revision 3 carries the episode through the split, keeps its original deadline, and suppresses a restart after the burst stops early. The included small `tests/recorded_split.json` fixture preserves the relevant recorded geometry.

Across the replayed interval, unique active warning episodes fell from 12 to 7. That is a diagnostic result, not a count of verified physical objects or proof that every suppressed warning was correct. Closely overlapping objects can still be confused. Separate synthetic tests verify that a disjoint new obstacle can warn while an earlier obstacle remains suppressed.

A separate regression moves a target back and forth for 10 seconds while changing its tracker ID every update. Only the initial burst is permitted. Tests also cover leaving/re-entering the warning distance, short dropouts, sensor outages, genuine absence/rearming, merging/splitting, early pulse termination, and watchdog expiry.

## Logging reduction

For the first 20 complete seconds, the original text occupies approximately **7.75 MB**. Replaying those events through compact logging produces approximately **219 KB of text / 29 KB gzip**, with no overload drops during that replay: roughly **97% less before compression**. This is a representative replay result, not a guaranteed size ratio or measured Pi disk/CPU benchmark. Numeric rounding, 2 Hz snapshots, omitted raw scans/repeated skeletons and reduced terminal duplication provide most of the reduction.

Tests verify ordered CV/LiDAR/HAPTICS groups, optional raw detail, final zero-command reporting, bounded overload behavior, disk-error containment and recovery of an atomic current-group checkpoint after forcibly killing its writer process. A checkpoint limits likely tail loss; it does not guarantee durability through storage failure.

## Navigation and camera checks

The suite exercises real pyzmq PUB/SUB delivery using **in-process transport**, stale-source invalidation, a deliberately unread subscriber, session/sequence handling, queued stale-source rejection and receiver expiry. A bind-conflict test verifies that the sensing controller continues and its burst expires normally. TCP and Unix-domain binding are restricted in this development environment, so actual cross-process TCP port 5555 still requires the supplied `test_nav_bridge.py` on the Pi.

Existing camera tests use fake models/frames and verify 720p geometry, OD/OD/pose scheduling, keypoint confidence/expiry, retained original timestamps and camera failure behavior. They do not measure YOLO accuracy or on-device throughput. `test_cv_only.py` is the on-device check for the webcam and both real model files.

## On-Pi acceptance checks

1. Run the full display without `--motors`. Confirm camera/LiDAR orientation and useful matches at the actual mount height.
2. Enable motors. Repeat the seated back-and-forth experiment for at least 10 seconds. Confirm one burst, at most 1.5 seconds, with no restart merely because you move closer or farther. Compare active encounter IDs and suppression records.
3. Introduce a genuinely separate obstacle at a clearly different bearing while the first remains visible; confirm that it can warn. Then remove one obstacle for at least 3 seconds of healthy observations and reintroduce it to confirm intentional rearming.
4. Stop with Ctrl+C. Confirm all physical motors stop and the log includes a successful shutdown zero-command event. A log command cannot verify vibration electrically.
5. Run CV-only and verify both `detect` and `pose`, actual resolution, source age and inference time.
6. Run sensing with `--nav`, then the Navigation test in another terminal. Stop sensing; the receiver must show invalid/expired data. With sensing continuing, stopping the receiver must not interrupt local warnings.
7. Repeat sustained tests with the actual Navigation and Voice workloads; record CPU, temperature, supply behavior and source ages. No 10–15 Hz inference guarantee has been established by these development tests.
