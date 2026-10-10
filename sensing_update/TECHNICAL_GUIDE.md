# Sensing Subsystem — Technical Guide (revision 7)

This standalone update uses your working Raspberry Pi virtual environment and existing YOLO26 models. It requests **1280×720 MJPEG at 30 FPS** from the EMEET C960 and uses **82.1° horizontal field of view** for a 16:9 image, as requested. 82.1° remains an estimate derived from the advertised diagonal field of view, not a measured lens calibration. It covers approximately −41.05° to +41.05° about the camera axis. Pixel-to-angle conversion uses perspective projection and the actual captured frame width.

## Installation and configuration

See `README.md` for the short installation and run commands. The Pi project directory is `/home/aarteam09/Sensing_SubS`. Extract revision 7 there to update the existing `sensing_update/` folder. The archive deliberately contains **config.defaults.json**, not `config.json`, so it cannot overwrite your mounting alignment or custom settings. The loader merges existing `config.json` overrides onto the new defaults. New installations use defaults automatically; copy the defaults to `config.json` only if that file does not already exist.

Camera resolution remains 1280×720, HFOV 82.1°, and inference size 416. OD and pose remain enabled. Add `--imgsz 640` only when evaluating its measured latency. `--no-display` works for the main application and CV-only test. `Q`, `Esc`, or `Ctrl+C` stops displayed tests. Use the venv interpreter explicitly because the copied activation script may still reference the old directory.

## Revision 3 fixes from the recorded run

- Split/merged LiDAR clusters inherit warning history using shared measured surfaces, alongside spatial and ID continuity. Merging warned encounters retains the earliest deadline. Known tracks retain their own history despite incidental overlap; new bridging tracks cannot unite independently observed histories. Brief back-and-forth motion and a new tracker ID alone do not rearm it. A return after confirmed departure does rearm it.
- Scan receipt intervals below 50 ms are marked timing-uncertain. Velocity requires two consecutive timing-valid observations, a plausible timestamp interval, and stable cluster geometry. Median range replaces nearest-point range for radial velocity; nearest returns still determine collision distance. Shape jumps reset velocity confidence and stationary patience.
- The controller waits only for the remaining portion of its requested 20 Hz period. Processing overruns skip the wait; they do not cause a catch-up burst. Actual Pi frequency is still workload-dependent.
- Compact logs replace duplicated scan arrays and repetitive terminal object lines. Warning start/end events, health transitions and shutdown zero-command outcomes are explicit. Detailed logging remains optional.
- NavBridge owns its socket in a separate worker, retries bind/import/send errors, and publishes fresh complete snapshots or invalid heartbeats. Navigation failure no longer stops sensing through an uncaught bridge exception.
- Added `test_cv_only.py` and `test_nav_bridge.py`. The latter is an example Navigation receiver and expiry test, not a route planner.

Your seated back-and-forth movement is real relative motion. A `MOVING` label is not itself evidence of a fault in that experiment. The warning history is now independent of `MOVING`/`STATIC`; classifying a person as static is not required to suppress repeat warnings.

## Haptic phases (revision 7)

Default `haptics.reminder_enabled=true` changes the finite-burst policy into an initial warning followed by a sustained low-duty reminder. A newly warned encounter keeps its original initial deadline, at most 1.5 seconds. Once that expires, `haptic_phase=reminder` commands `duty_override=0.25` for currently eligible threats. ID/surface continuity and merge history do not restart the strong burst. Sensor loss terminates the initial phase early; without a previously confirmed departure, a returning known hazard resumes only the reminder.

Duty overrides use angular interpolation normalized to a peak of the requested duty. They bypass `min_duty`, so the dominant directional motor receives 25% and its neighbor no more than 25%. A separately triggered initial warning may command higher output on shared motors. The separate heartbeat watchdog remains active for reminders.

Departure is confirmed after `departure_confirm_s=0.5` of observed geometry outside both the close bubble and walking warning corridor, or the existing `rearm_absence_s=3` of healthy absence. Missing camera support alone never qualifies. The two pulses use `departure_pulse_s=0.3`, `departure_gap_s=0.25`, and `departure_duty=1.0`, on the single motor nearest the last warned bearing. Pulse segments have individual expiry deadlines. Sensor failure cancels cues and resets clearance timers; a hazard claiming the same motor defers an unstarted cue or cancels a started cue. A cue indicates a relative departure observation, not absolute object motion or a completely safe path. Healthy absence cannot distinguish true departure from occlusion/detection loss.

Each active record retains the backward-compatible `active_bursts` field and carries `haptic_phase` (`initial`, `reminder`, or `departure`). `suppressed_ids` now means full-burst suppression; those IDs can still receive reminders. Phase transitions and departure confirmations/cancellations are logged immediately. Compact track snapshots remain sampled.

Set `reminder_enabled=false` to select the previous silent-after-burst behavior, retained for compatibility. Default tests of that legacy policy explicitly select it; new tests exercise the sustained default separately.

## Warning selection

The old LiDAR-only implementation warned for nearby returns across the front sector. The default here is `haptics.policy = "fusion"`:

| Situation | Default behavior |
| --- | --- |
| Fresh camera detection matches LiDAR returns in the forward corridor | Warn below 1.5 m using the matched returns |
| Fresh camera sees no matching object | Suppress normal-range unknown returns |
| Any valid front cluster is within 0.6 m | Warn even without camera confirmation |
| Camera/inference becomes unavailable or stale | Use LiDAR corridor warnings below 1.5 m |
| LiDAR data becomes unavailable or stale | Clear directional motor output |
| Track is only coasting after its returns disappear | Keep tracking briefly, but do not actuate from that old track |

The forward corridor is 0.45 m to each side of the sensor axis. These distance/width values are configurable **prototype tuning values**. A close bottle, cable, mounting part, or your body can still generate a real return and the close fallback will warn. Camera matching is angular evidence, not proof that a return belongs to the detected object. A wall can be missed by a detector trained on object classes; suppressing unknown normal-range returns trades nuisance reduction for missed hazards. The close/failure fallbacks reduce that tradeoff but cannot eliminate it.

Static obstacles can still cause collision warnings. The 2/7 second static-classification timers do not delay immediate haptic feedback.

Motor angle and strength mapping retains the five-motor design:

| Position | Angle | BCM GPIO | Physical pin |
| --- | ---: | ---: | ---: |
| Far left | −90° | 17 | 11 |
| Left | −45° | 27 | 13 |
| Center | 0° | 22 | 15 |
| Right | +45° | 23 | 16 |
| Far right | +90° | 24 | 18 |

Angles between motor directions blend the adjacent motors. Strength increases toward 0.4 m, with a 0.18 minimum running duty and 200 Hz PWM. The module's 5 V supply remains separate from the GPIO control signal, with common ground. This update does not change wiring.

## Align your mounted sensors

Open `sensing_update/config.json`. No calibration settings are written automatically.

- `lidar.front_offset_deg`: raw LiDAR angle pointing straight ahead. Example: if a target ahead produces raw angle 90°, set this to 90. The corrected angle is raw angle minus this offset.
- `lidar.reverse`: change to `true` only if targets on your right appear on the left after setting the forward offset.
- `camera.yaw_deg`: camera pointing angle relative to the same forward axis; positive means right.
- `camera.x_m`, `camera.y_m`: camera position relative to LiDAR, in metres; right is +x and forward is +y. Measure these if the sensors are apart.
- `camera.hfov_deg`: 82.1 by request. Only use this estimate for the intended 16:9 mode until calibrated.

Use one opaque target about 1 m away: first directly ahead, then left and right within the camera image. Keep the assembly level and stationary. Check that the camera detection and LiDAR cluster correspond in the window. The viewer displays a rolling 2D scan, not a persistent room map. Head motion changes the scan plane; alignment settings do not compensate for pitch/roll.

## Camera resolution and inference

Native capture increases from 640×360 to 1280×720. YOLO's `imgsz` defaults to 416 to preserve your existing CPU performance. A sharper native camera image does not make the neural network run at 720p. To test a larger detector input separately:

```bash
QT_QPA_PLATFORM=xcb ./venv/bin/python -u sensing_update/main_sensing.py --models cv --imgsz 640
```

Compare the displayed inference time and result age. Higher network input sizes cost CPU time and may trigger vision expiry/fallback on the Pi. The capture thread continuously replaces the latest frame, while inference works separately. **Both tasks are enabled by default:** `detect → detect → pose`, repeating. Only one model runs per inference cycle, using your existing `cv/yolo26n.pt` and `cv/yolo26n-pose.pt`. `camera.pose_every=3` means every third cycle is pose; 0 explicitly disables pose, while 1 would run pose only and lose general object updates. Missing/failed pose weights produce a visible status and allow OD to continue.

Pose inference returns person boxes and skeletons together. Those boxes are used directly for person fusion, while recent non-person OD results retain their original timestamps and still expire normally. OD cycles attach recent skeletons to person boxes using one-to-one IoU matching and a 0.7 s pose age limit. Every keypoint retains its confidence. Skeletons are drawn in the camera panel and timestamped pose evidence is attached to matching LiDAR tracks. This box association is a prototype method, not a validated identity tracker.

Human tracks can display `APPROACHING_RELATIVE`, `RECEDING_RELATIVE`, or `LATERAL_RELATIVE` from recent LiDAR velocities when camera-motion confidence permits. These describe motion relative to the wearer; walking toward a stationary person also reduces range. Pose exposes confident joints/torso visibility, but facing direction and predicted intent stay `UNKNOWN`: this package does **not** claim that 2D skeletons already implement reliable human trajectory prediction. Intent modeling needs temporal validation and a better head-motion reference.

A pose checkpoint does not perform general multiclass object detection. The two separately loaded checkpoints also do not share their forward-pass computation. Their schedule and Pi timings must be measured, especially after adding speech recognition and navigation. See `HARDWARE_REVIEW.md` for the integration review.

## Tracking, timing and subsystem requirements

LiDAR acquisition runs in a supervised child process so a blocked serial driver cannot directly block YOLO or GPIO cleanup. The driver's asynchronous output queue is bounded. Complete revolutions are assembled with scan-boundary handling before invalid-return filtering, and empty completed scans clear current obstacles. Old partial revolutions are discarded. Scan timestamps are receipt-time midpoint estimates, not hardware timestamps.

Clustering preserves nearest returns. The kinematic tracker uses one-to-one assignments and per-track timestamp differences; the separate warning memory handles many-to-many split/merge lineage. Brief semantic memory prevents a single missed detection from immediately erasing a class. Semantic memory does not keep old camera confirmation alive for warnings. Matching checks result age and camera/scan timestamp skew and applies configurable horizontal sensor offsets.

Forward ego-speed compensation is accepted only when at least three eligible tracks span at least 20° with consistent radial speeds. Moving-capable classes such as people and vehicles are excluded as speed references. An empty/insufficient reference set resets ego validity. Background optical flow flags uncertainty; it is not an IMU. Motion labels are `UNKNOWN` until supported; valid stationary evidence must last 7 seconds for people and 2 seconds for other objects. This is limited forward-only compensation, not full 6-DOF odometry or validated moving/static classification during turns.

The warning loop is independent of YOLO, runs at a requested 20 Hz, and expires LiDAR scans at 0.4 s and camera results at 0.6 s. A separate motor watchdog clears PWM after a 0.35 s control heartbeat gap. Shutdown attempts to zero and close every motor before waiting for sensor workers. These are software targets, not measured end-to-end guarantees; measure actual hazard-to-vibration latency and long-run reliability on the mounted Pi. A software watchdog cannot protect against a frozen operating system or failed GPIO hardware.

Navigation communication defaults to **off**. Enable it with `--nav` or `navigation.enabled=true`. See the detailed protocol below. Immediate haptic decisions remain independent of Navigation and its static-map filter.

The project report's response-time, 3000 SAR cost, 2 kg mass and at-least-5-hour battery requirements still need physical verification. This package implements the sensing software changes; it cannot demonstrate cost, mass, runtime, crowd accuracy or complete system compliance. A head-mounted 2D LiDAR only sees its current plane and does not guarantee detection of steps, drop-offs or hazards below that plane.

## NavBridge: architectural boundary and protocol

NavBridge separates local obstacle reactions from route planning. Its four engineering benefits are:

1. **Fault containment between subsystems.** Navigation can stop or lag without requiring a reply before sensing continues. Python-level bridge errors are caught in the bridge worker. This does not isolate common power, kernel, memory exhaustion or native-library failures on the same Pi.
2. **Bounded asynchronous work.** The controller replaces a single latest-state slot. A dedicated thread serializes JSON and owns the PUB socket, with send high-water mark 2, non-blocking sends and zero linger. There is no unbounded application backlog. Work still consumes CPU and has latency; it is not zero-cost or a hard-real-time guarantee. PUB drops messages when subscribers cannot keep up and can lose startup messages.
3. **A calmer static obstacle layer.** Only currently observed tracks with supported `STATIC` state enter `NAV_STATIC`: at least 7 seconds of stationary evidence for people, 2 seconds for other objects. This reduces transient entries in the persistent costmap. Dynamic hazards still need immediate avoidance and, eventually, a dynamic local planner; filtering them from the static topic does not make them harmless. A* and Dijkstra are not inherently unable to operate in changing environments; replanning policy matters.
4. **An explicit cross-language contract.** Python or C++ can subscribe and decode the same topic and JSON. A ROS2 team can write a ZeroMQ-to-ROS2 adapter. ROS2 does not consume this custom protocol automatically.

Default endpoint: `tcp://127.0.0.1:5555`, **same Pi only**. Publisher binds; Navigation connects and subscribes to the exact prefix `NAV_STATIC `. Each message is **one UTF-8 frame**: topic, one ASCII space, then JSON. Do not expect multipart frames. Publication is requested at 4 Hz, independent of the 20 Hz control loop.

| Field | Meaning and receiver action |
| --- | --- |
| `schema_version: 3` | Reject unsupported versions. |
| `session_id`, `sequence` | New UUID for each publisher instance; increasing message counter. Reject duplicates/out-of-order messages. Reset the costmap source on a new session. |
| `frame: body_x_right_y_forward` | Metres relative to the calibrated sensor frame: +x right, +y forward. Angles are degrees, positive right. These are **not global/map coordinates**. |
| `timestamp` | Publisher wall-clock Unix seconds, for diagnostics. |
| `source_monotonic_s`, `source_age_s` | Local source-time estimate and its age at send. Monotonic timestamps cannot be compared across separate computers. |
| `snapshot: true` | Replace this source's previous complete snapshot, including when the list is empty. Do not append indefinitely. |
| `valid`, `status` | Invalid means missing/stale/failed input or shutdown, not proof of a clear path. Enter the Navigation degraded-data policy. |
| `ttl_s: 0.75` | Expire locally after receipt even if no more messages arrive. Do not freeze old obstacles as fresh. |
| `static_obstacles` | Each entry: track `id`, `semantic_class`, `x_m`, `y_m`, nearest `dist_m`, and median `angle_deg`. |
| `ego_velocity_mps`, `ego_velocity_valid` | Limited forward ego-motion estimate; do not treat it as full odometry. |

Cluster x/y values are component-wise medians, angle is median bearing, and distance is the nearest return: these statistics need not describe the exact same point. Track IDs can change; they are scoped to the stream/session and are not physical-person identities. Warning encounter IDs are separate from Navigation track IDs.

The publisher invalidates data older than 0.4 seconds even if its controller stops updating, and attempts a final invalid snapshot on shutdown. Delivery of that final message is not guaranteed: the receiver must enforce TTL independently. `test_nav_bridge.py` demonstrates schema checking, session/sequence handling, replacement and expiry. By default it also compares `source_monotonic_s` with its own same-host monotonic clock, rejecting data delayed in a socket queue by more than 0.4 seconds. This comparison is valid between processes on the same Pi; accepted data also expires when its source age reaches 0.4 seconds, even if the 0.75-second receipt TTL has not elapsed. Its receive-time TTL is an additional liveness limit; deployment across machines additionally needs a bounded-delay policy and synchronized source-age checks. A TTL beginning at receipt alone cannot detect arbitrary network transit delay.

Navigation must transform body-frame data using time-aligned sensor mounting, heading and position before inserting it into a map. A head-mounted sensor's direction is not necessarily the walking direction. Voice Commands should communicate intent to Navigation separately. Neither subsystem should directly overwrite sensing motor GPIO; shared haptic/audio output needs a single owner and explicit arbitration. ZeroMQ does not reserve CPU resources for any subsystem: profile sustained OD + pose + Navigation + speech workloads together.

Official references: [ZeroMQ socket semantics and thread ownership](https://libzmq.readthedocs.io/en/latest/zmq_socket.html), [PUB/SUB reliability and slow-subscriber limitations](https://zguide.zeromq.org/docs/chapter5/).

## Smaller, recoverable Sensing_LOG

Each run produces timestamped files under `sensing_update/Sensing_LOG/`. Every group covers 10 seconds, in this order: **CV → LiDAR → HAPTICS → TERMINAL**. Each record has a wall time and elapsed receipt time; original source timestamps remain available where relevant.

Default `compact` mode keeps:

- Every CV result's metadata, object classes, boxes, confidence, angles, inference time, task and pose status; pose timestamp and visible-keypoint count replace repeated full skeleton arrays.
- Fused track snapshots at 2 Hz plus health/timing-quality changes immediately. These contain IDs, angles, distance, class, motion and confidence flags. Raw point clouds are omitted.
- Immediate warning encounter start/end transitions, selected hazard angles/distances/reasons, suppressed IDs, and motor duties sampled at 2 Hz. Duty order is **FL, L, C, R, FR**, directions **−90, −45, 0, +45, +90 degrees**; GPIO order is in the recorded configuration.
- Terminal errors and status messages. Routine object lines, repeated active-warning lines and repeated suppression lines are already represented structurally and are omitted from the terminal section, with counters.
- A final HAPTICS `shutdown` event with the zero-command outcome. This records software/API success, not physical motor readback.

Compact mode intentionally does **not** preserve every intermediate PWM adjustment, raw return or keypoint. For a short, deeper investigation use `--log-mode detailed`. That mode keeps every CV numeric result, every control-cycle track/haptic record and each newly received raw completed scan once; it still avoids the duplicate calibrated scan array. Neither mode stores images, video or audio.

The writer uses a bounded queue (2048 events) and a 10-second buffer capped at 10,000 records. Producers do not wait for disk. Overload drops/evictions are counted in the log; transition events can evict queued telemetry when full. Even critical events can be lost under sustained extreme overload or disk failure—this is observable best-effort logging, not a certified lossless flight recorder. `[LOG ERROR]` remains visible and sensing continues when the writer fails.

Completed groups are flushed and fsynced. Every second the current incomplete group is atomically replaced in a `.pending` checkpoint. On normal shutdown it is committed and the checkpoint is removed; text logs are compressed to `.log.gz` using low-cost gzip after control has stopped. Parts rotate at a 10-second boundary after approximately 25 MB of uncompressed text (a part can exceed the limit by one group). Existing logs are not automatically deleted.

After abrupt termination, keep **both `.log` and `.pending`** files; the checkpoint may contain the final group not yet committed. If its group number already exists in the text log, treat the committed group as authoritative. A power failure can still lose the newest uncheckpointed data or affect the storage device; checkpoints and fsync are not absolute durability guarantees. If compression fails, keep the original text log.

To read a completed compressed file without making another copy:

```bash
zcat sensing_update/Sensing_LOG/Sensing_LOG_YYYYMMDD_HHMMSS_microseconds.log.gz | less
```

Configuration keys under `logging`: `mode`, `snapshot_hz`, `queue_size`, `checkpoint_s`, `compress_on_close`, `max_part_mb`. The grouping interval is 10 seconds. Snapshot Hz is configurable; documented 2 Hz is the default.

## Verification and limits

See `VALIDATION.md` for test results and recorded-run measurements. The tests exercise geometry, dropouts, motor command lifetime, OD/pose scheduling, recorded cluster splitting, oscillating targets, uncertain scan timing, log overload/crash checkpoints, and ZeroMQ publish/subscribe behavior. They do not validate the Pi's actual motor wiring, model accuracy, mounting or combined Navigation/Voice performance.

Run from the Pi:

```bash
cd /home/aarteam09/Sensing_SubS/sensing_update
../venv/bin/python -m unittest discover -s tests -v
```

A hardware-free display is available with `main_sensing.py --demo`; it cannot enable physical motors. `--snapshot preview.png` renders one simulated image.


## Revision 4 identity and timing correction

Encounters retain generation-qualified track IDs across geometry replacement and merges. Identity participates in the many-to-many assignment before the one-to-one spatial fallback; proximity competition cannot strip a continuously known track's history. IDs are scoped to acquisition generation; after an encounter expires its aliases expire too. Shared surfaces still merge histories conservatively, which can suppress distinct touching objects. This is not permanent physical identity recognition.

CV records add `frame_to_result_ms`, `pre_inference_age_ms`, `postprocess_ms`, `result_interval_ms`, and `stale_frames_skipped_total`. The existing `inference_ms` includes model execution and output conversion; it is not pure neural-network compute time. `finished` now includes background-motion postprocessing, and freshness is evaluated at that later time. Frame timestamps are host capture receipt times, not sensor exposure timestamps. Camera buffering is not measured.

Each LiDAR log record includes `control_cycles_since_record`, `max_control_gap_ms`, and `vision_unhealthy_cycles_since_record`, aggregated over every control call since the prior record, even when intermediate snapshots are omitted. These do not measure physical vibration latency. Compact logs remain sampled for tracks and cannot reconstruct every clustering decision. Overload counters must be checked when interpreting missing events. The display remains enabled by default; headless operation is still optional.


## Revision 5 display and diagnosis

`cv/display_labels.py` assigns deterministic bright BGR class colors and per-class left-to-right numbering. The inference worker decorates copies after OD/pose cache association, so the original class labels, timestamps and pose caches remain unchanged. The main and CV-only windows share the same annotation function. Display numbers are not track IDs and never influence fusion or haptics.

`version.py` defines 8.0; `--version`, startup output, display and log header identify the code. Each haptic record includes `encounter_by_track` and `encounter_events`. Encounter creation, merge, first-warning and rearm events trigger immediate log submission even between sampled snapshots. These are bounded asynchronous records: overload and disk errors still matter, and logs cannot prove physical object identity or actual vibration.


## Revision 6 current behavior

This section supersedes historical statements about a persistent obstacle remaining silent after its initial warning. Software version is 6.0. Continuous reminders increase motor runtime compared with the old policy; actual battery life and vibration response must be measured with the full prototype. No GPIO reassignment or sensor close-return exclusion has been made.


## Revision 7 departure changes (retained)

The uploaded version-6 log shows ten departure confirmations and eight cancellation records. Seven cues were cancelled before any commanded departure output; one was interrupted after playback began. Only two cues show both pulse stages in the recorded active feedback. Global hazard cancellation was the main reason cues were not felt, with short low-duty segments a possible additional factor.

Defaults now use two 300 ms / 100% duty pulses with a 250 ms gap. Cues use the nearest of five fixed directional bearings, so a single motor carries the pattern. Other directional warnings continue. Current hazard motor occupancy (computed before adding cues) has priority. An unstarted cue is deferred for at most `departure_wait_s=3`; an already started cue is cancelled when its motor is claimed by a hazard. The same encounter returning cancels its cue. Cues on one motor are serialized across both pulses and their silent gap. Sensor loss still cancels every cue.

New stage, deferred, expired, completed and incomplete events distinguish policy decisions from output strength. A completed event requires both pulse stages to have been commanded; it is not measured vibration or proof of exact pulse lengths. A missed second stage is logged as incomplete. `test_departure_pattern.py` provides a standalone one-motor test, dry-run by default, and clears commands on normal exit, Ctrl+C or SIGTERM.

This section supersedes revision-6 pulse values and its global cancellation rule. Initial warnings and 25% reminders are unchanged.

## Revision 8 encounter correction (current)

Confirmed departure now rearms the initial warning on subsequent eligibility, even if the original LiDAR track ID remains alive. The log emits `rearmed_after_departure` followed by `first_warning` with a new deadline. The departing cue is cancelled on return. Cue playback or completion is not required for rearming: geometric departure confirmation is the trigger.

Matching first assigns every known `(generation, track_id)` to its existing encounter. Unknown tracks may inherit shared-surface history, but cannot merge histories of independently observed owners. With no directly observed owner, measured split/merge continuity still keeps the earliest warned deadline. Class names and numbered display labels do not participate. These rules reduce erroneous warning suppression without claiming physical object identification; overlapping or occluded objects remain geometrically ambiguous.
