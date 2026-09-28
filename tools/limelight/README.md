# Limelight 3A desktop recordings

Record AprilTag **detection data**, not video or images. The recorder needs only
Python 3.9+; no robot, Java, OpenCV, or custom camera calibration is needed to collect data.
It makes read-only HTTP requests and does not switch pipelines or change camera settings.

## Connect and record

1. Connect the 3A directly to your computer with a USB data cable and let it boot.
2. Open `http://limelight.local:5801` (or use the address shown by Limelight Hardware
   Manager). Select an AprilTag 36h11 pipeline and enable corner output (`pts`).
3. From this repository, run:

```sh
python3 tools/limelight/record.py record \
  --scenario red-front --alliance red --side audience \
  --expected-state upright --notes "Stationary, camera approximately level"
```

Press Enter when positioned, move/hold the camera as planned, then **Ctrl+C to save**.
Each invocation creates a new timestamped session under `recordings/limelight/`.
The terminal shows detected IDs, corner availability and repeated-frame status.
If `limelight.local` does not resolve, add `--host CAMERA_IP`; REST defaults to port
5807 (the browser UI's 5801 is a different service). Use the camera IP, not the
computer's USB network adapter IP.

For a timed recording, no Enter prompt is needed:

```sh
python3 tools/limelight/record.py record \
  --scenario blue-angled-moving --alliance blue --side audience \
  --expected-state upright --duration 30 --notes "Cell fixed upright; sweep camera left to right, then approach"
```

Optional flags:

- `--distance 2.0`: independently measured distance **from camera to cell opening**,
  meters, for a stationary session only. Do not use a Limelight estimate as ground truth.
- `--mount handheld|fixed|turret`, `--notes "..."`: describe setup and lighting.
- `--pipeline-index 0`: also read/save the specified pipeline's configuration;
  does not activate it. Set this to the pipeline you selected in the UI.
- `--hz 30`: maximum polling rate; not a promise to capture every camera frame.
- `--calibration FILE`: embed a verified replay calibration file (optional).
- `--output PATH`: store sessions elsewhere.

Keep resolution, crop/zoom, exposure and calibration constant during a session.
Start a new session when changing them; snapshots are taken at the start only.
Use notes to record firmware and the active calibration shown in the UI if a
firmware version does not expose those fields through the saved status reports.

## Calibration: baseline first, accuracy testing after

Limelight ships with default calibration. Not having performed custom calibration
does not prevent recording or initial detection tests. Make a short baseline now;
for quantitative distance/aim testing, use the built-in ChArUco calibration workflow
and then compare against independently measured stationary targets.

Use a flat, accurately measured board. In supported Limelight OS versions, create a
ChArUco Calibration Preview pipeline, enter its board dimensions/dictionary, capture
at least 25 diverse views including tilted and edge-of-image views, then calibrate.
Save/upload the result and verify that custom calibration is active in the UI.
Follow the [official calibration guide](https://docs.limelightvision.io/docs/docs-limelight/getting-started/performing-charuco-camera-calibration)
for your installed firmware's exact controls.

The recorder attempts to save default, file, and EEPROM calibration plus status and
hardware reports. Unsupported endpoints are recorded as errors without preventing
capture. These snapshots preserve evidence; their presence alone does **not** prove
which calibration was active. No calibration values are guessed automatically.

Pixel corners can be reprocessed with improved intrinsics later if the pixel mapping,
resolution and optical setup match. Previously reported Limelight 3D poses do not
magically change. Detection recordings cannot rerun the detector with different
exposure/threshold settings, recover missed tags, or replace ChArUco calibration images.
Camera-to-shooter mounting calibration is a separate, later step.

Lighting tuning is separate from both lens and mounting calibration. The previous
season's `LimelightCalibration.java` (on `master`) searches exposure, gain and image
processing settings. A BIOBUZZ version should score detection availability over
fixed time windows, cell-opening angle/range jitter while stationary, and tracking
through motion. Counting only successful detections hides dropouts; repeated polls
must not count as independent observations. A moving-camera test should not minimize
raw position variance, because the true position is changing. Record each settings
candidate separately and compare acquisition/loss, frame age and stationary accuracy.
Saved detection data can compare runs, but evaluating a new exposure requires a new
live capture. See [Limelight's exposure guidance](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags).

## Suggested sessions

Use the [numbered capture guide](SCENARIOS.md) for exact camera positions, distances,
motion timing, recording commands and expected observations. The recorder does not
automatically advance through that checklist; run one labeled recording per take.

| Scenario | What to observe |
| --- | --- |
| Red/blue, facing directly, stationary | Identity, aiming signs, distance repeatability |
| Known distances, near and far | Range bias; record measured camera-to-opening distance |
| Left/right oblique views | Pose stability and changing tag visibility |
| Targets near image edges | Calibration/distortion effects |
| Upright vs inverted cell | Correct target rejection |
| Slow sweep, fast sweep, approach/recede | Lag, repeated frames, motion-related loss |
| One, two, then four tags visible | Partial-visibility behavior |
| Both cells visible; opponent tags visible | Alliance and side filtering |
| Cover lens / unplug and reconnect | Missing data, recovery and stale results |

A wall-mounted tag board is useful for detection and corner tests. To validate
opening position and shootability, it must reproduce the SDK's physical tag spacing,
size and orientation; real hive geometry is better. Label ordinary wall boards as
such so their inferred opening positions are not mistaken for measured ground truth.

## Saved data and inspection

- `session.json`: scenario, alliance/side, notes, expected state/distance, and device snapshots.
- `frames.jsonl`: one measurement record per HTTP poll, including the complete JSON
  response, computer timestamps, request duration, repeat status and errors. Despite
  its filename, this contains **no camera images**.
- `frames.csv`: spreadsheet-friendly raw detection/latency summary (Limelight `tx/ty`
  are its selected target offsets, **not** our computed cell-opening offsets).
- `summary.json`: poll counts, distinct observed frames, errors and tag/corner statistics.

Records are flushed line by line; Ctrl+C closes the file and saves metadata. A hard
kill may leave metadata marked incomplete; earlier complete JSONL rows remain usable.
Inspect ignores an unterminated partial final row; remove that row before Java replay.
Counts are labeled as polls because several polls may refer to the same camera frame.
Repeated identity uses camera timestamps/frame index; firmware lacking those fields
has unknown repeat status. A camera restart resets this tracking when its identity changes.

Recreate CSV and statistics later:

```sh
python3 tools/limelight/record.py inspect recordings/limelight/SESSION
```

## Replay through our Java targeting class

Requires JDK 17+ and the host OpenCV test dependencies downloaded by the existing
runner. Supply a JSON calibration with these fields:

```json
{
  "fx": 700.0, "fy": 710.0, "cx": 640.0, "cy": 360.0,
  "distortion": [],
  "corner_order": [0, 1, 2, 3]
}
```

**Those numbers and corner order are a synthetic test example, not a 3A calibration.**
Replace them with the actual intrinsics scaled to the detection pixel resolution,
distortion coefficients, and verified decoded-tag corner order. See
[targeting setup](../../TeamCode/LIMELIGHT_TARGETING.md). The camera's downloaded
calibration JSON has its own format; this file is the normalized input for our solver.

```sh
python3 TeamCode/tests/run_targeting_tests.py \
  --replay recordings/limelight/SESSION --calibration my-camera.json
```

This writes `targeting.csv`: aiming X/Y, range, 3D opening position, roll, fit error,
and acceptance per poll, plus distance error when a measured distance was supplied.
Use `--output other.csv` to compare solver/calibration revisions without overwriting
an earlier result. The default output is overwritten on each replay; source JSON is retained.

Replay uses elapsed duplicate-frame age from the recording rather than today's wall
clock, plus recorded camera latency. It is not a timing-accurate simulation of the
Control Hub: historical robot parsing latency is unavailable, network transport is
reported separately, and the first observation of an already-stuck frame cannot be
aged accurately. Replays do not automatically infer correctness from scenario labels.

## Tests

```sh
python3 -m unittest discover -s tools/limelight -p 'test_*.py'
python3 TeamCode/tests/run_targeting_tests.py
```

The Python suite uses a local mock HTTP camera to test recording, duplicate/stale
frames, malformed responses, disconnections, session metadata and CSV output.
Actual 3A connectivity and firmware response details still need a hardware session.

References: [3A USB setup](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a),
[REST API](https://docs.limelightvision.io/docs/docs-limelight/apis/rest-http-api),
[detection JSON](https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification).
