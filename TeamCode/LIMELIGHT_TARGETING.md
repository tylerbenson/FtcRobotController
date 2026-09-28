# Limelight cell shooting-position example

**Odometry handles aiming and shot distance.** `LimelightCellTargeting` only reports
which cell of our alliance appears upright: `AUDIENCE_UP`, `SCORING_TABLE_UP`, or
`UNKNOWN`. Odometry remains responsible for aiming angles and range.

## Required setup

- AprilTag 36h11 pipeline with decoded corner (`pts`) output enabled.
- Allow the relevant BIOBUZZ IDs (30–45 for all cells).
- Camera mounted upright, without roll, with a view toward the hive. Test the full
  intended pitch/viewing-angle envelope, including both hive states.
- At least **two fully detected tags from the same cell**. Any two member IDs work.
- Verify actual upright/inverted states from all intended camera positions on the 3A.
  The classifier uses the SDK's known left-to-right ordering of member IDs.

No camera matrix, lens distortion coefficients, measured tag spacing or camera-to-
shooter offset is needed by this classifier. Default lens calibration is sufficient
to begin these tests. Exposure/gain tuning can still help reliable detection during
motion. Neither custom lens calibration nor captured sample data is required.

## Control Hub live test

`TestLimelightCellState` appears in the Driver Station TeleOp list as
**Test: Limelight Cell State** (group **Test**). It only uses the camera; no motors,
turret, odometry or other robot hardware are needed.

1. Connect the Limelight 3A by USB to the Control Hub. In the active robot
   configuration, add/name it **limelight**.
2. Configure Limelight pipeline **0** as described above, allowing **both alliances'
   IDs 30–45**, with corner output enabled. If using another slot, change
   `APRILTAG_PIPELINE` in `TestLimelightCellState.java` before deploying.
3. Build/install the Robot Controller app on the hub and select this TeleOp.
4. Press **INIT** for live telemetry; **Play** continues the same display.
   Move the camera between red and blue cells and between field sides. Keep the
   camera upright. Press **STOP** to stop camera polling.

Separate **RED cell UP** and **BLUE cell UP** lines report `AUDIENCE_UP`,
`SCORING_TABLE_UP` or `UNKNOWN`. Each alliance uses the same camera frame but
maintains independent confirmation. Evidence lines show the reason, supporting
tag count, confirmation frames and estimated frame age. The ID list shows the
latest received detections (which may be stale; the classifiers reject stale data).

Expect a visible cell to confirm after at least three new frames spanning 100 ms.
An alliance out of view returns `UNKNOWN`; its previous state is not retained when
fresh detections no longer support it. Frozen/disconnected results expire after
200 ms. Seeing only the inverted cell does not establish the unseen cell's state.
Cover tags until only one remains and check that confirmation clears; uncover them
and check recovery. Also test both alliances together, camera motion, and USB loss.
These are vision observations, not a complete permission to fire.

## OpMode integration

```java
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
LimelightCellTargeting cells = new LimelightCellTargeting(
        limelight, LimelightCellTargeting.Alliance.RED);
limelight.pipelineSwitch(aprilTagPipelineIndex);
limelight.start();

// Each OpMode loop:
LimelightCellTargeting.Observation observation = cells.update();
// Determine this from your robot/odometry logic, not camera visibility alone.
LimelightCellTargeting.Side shootingSide = sideSelectedByOdometry;
boolean visionAllowsShooting = observation.isUp(shootingSide);
boolean mayFeed = visionAllowsShooting && odometryAimReady && shooterReady;
// Apply mayFeed to your existing feed controller every loop; false must stop feeding.
// Odometry/shot model remains responsible for heading, distance and wheel speed.
telemetry.addData("Cell up", observation.state);
telemetry.addData("Vision reason", observation.reason);
telemetry.addData("Supporting tags", observation.supportingTags);

// In finally/stop:
limelight.stop();
```

Names such as `sideSelectedByOdometry`,
`odometryAimReady`, and `shooterReady` represent your existing/configured robot logic.
The example deliberately does not select a permanent field side or command motors.

## How classification works

1. IDs group detections by alliance and audience/scoring-table cell.
2. Average each tag's four corners to get its image center. Corner list order is
   irrelevant. Compare centers in increasing tag-ID order: predominantly left-to-right
   means upright; right-to-left means inverted. The direction must be within 60 degrees
   of horizontal and centers at least 4 pixels apart.
3. Every available pair must agree. Contradictory pair directions or two upright
   cells produce `UNKNOWN`. Fewer than two tags cannot directly confirm a cell.
4. Confirmation requires at least **3 distinct frames spanning 100 ms** by default.
   Duplicate timestamps do not advance confirmation. Missing, contradictory or stale
   data clears confirmation immediately, including when the previously known side changes.
5. Observations older than **200 ms** are rejected. The class also ages frozen camera
   timestamps even if the SDK keeps receiving the same frame over HTTP.

The extended constructor configures frame count, confirmation time and age limit.
These are starting values for testing, not validated competition thresholds. The
classifier requires a positive camera timestamp from the SDK.

`Observation` includes state, reason, supporting tag count, consecutive frame count
and estimated age. `isUp(side)` is false for `UNKNOWN`. Never retain an old true gate
when a newer observation says unknown. Never infer the unseen opposite cell is up
solely from an inverted tag. Call `reset()` when changing pipeline or camera setup.

This uses multiple tags as redundant orientation evidence, **not** as a joint 3D
cluster solve. A consistent upright image over several frames does not prove that
the hive has reached its mechanical stop. Validate transition behavior and apply
any additional settling delay/interlock in the feed controller. Roll/perspective can
invalidate the image-orientation heuristic; unsuitable viewpoints must not be used
for firing authorization. A low-confidence scene should leave the gate closed.

## Tests

```sh
./gradlew :TeamCode:assembleDebug
python3 TeamCode/tests/run_targeting_tests.py
```

The test runner requires Python 3 and JDK 17+ and downloads pinned FTC SDK test
dependencies on the first run. No camera is needed for these synthetic tests.
Tests cover all cells, partial visibility, inversion, conflicts, distinct-frame
confirmation, time windows, disconnects, frozen data, restarts and arbitrary corner
permutations. Hardware validation remains outstanding.
