# Limelight cell targeting

`LimelightCellTargeting` reads a Limelight AprilTag frame and estimates the BIOBUZZ
cell opening relative to the camera optical center. It groups tags by alliance/cell
and jointly solves their corners with OpenCV. It does not need a webcam or VisionPortal.

## Setup

1. Configure a 36h11 AprilTag pipeline and enable corner (`pts`) output in Limelight.
2. Supply the camera's `fx`, `fy`, `cx`, `cy` and OpenCV distortion coefficients for
   the exact pipeline resolution and crop. Input corners must use absolute pixels,
   origin at top left, X right and Y down. Convert coordinates upstream if your
   firmware returns a different convention. Do not apply distortion correction twice.
3. Verify the corner list order with an upright tag, then rotate the tag and verify
   that the indices follow the decoded tag rather than screen position. Pass the
   indices corresponding to bottom-left, bottom-right, top-right, top-left.
   There is deliberately no guessed ordering or camera calibration.
4. Mount the camera without roll and looking toward the shooting target. Pitch is
   allowed, but steep pitch makes the camera-relative upright heuristic unreliable;
   validate both hive positions from all intended shooting locations.

The Limelight's tag-size or field-map pose settings do not affect this solver: it
uses raw corners and SDK 12's 3.25-inch tag geometry. Camera calibration still matters.

## Usage inside an OpMode

```java
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
limelight.pipelineSwitch(aprilTagPipelineIndex);
limelight.start();

// These variables must come from your calibration and corner-order verification.
LimelightCellTargeting.Calibration calibration =
        new LimelightCellTargeting.Calibration(
                fx, fy, cx, cy, distortionCoefficients, sdkCornerToLimelightIndex);
LimelightCellTargeting targeting = new LimelightCellTargeting(
        limelight, LimelightCellTargeting.Alliance.RED, calibration);

// Each loop; use the known field side when available.
Optional<LimelightCellTargeting.Target> result =
        targeting.update(LimelightCellTargeting.Side.AUDIENCE);
// targeting.update() allows either side and chooses the nearest valid upright cell.
if (result.isPresent()) {
    LimelightCellTargeting.Target target = result.get();
    telemetry.addData("Aim X / Y (degrees)", "%.2f / %.2f",
            target.xDegrees, target.yDegrees);
    telemetry.addData("Distance (meters)", target.distanceMeters);
    telemetry.addData("Cell / visible tags", "%s / %d", target.side, target.visibleTags);
    // Send target.xDegrees to your turret controller and range to your shot model.
    // Check freshness, settling and shooter readiness before feeding a ball.
} else {
    // Clear aiming/feeding requests. Do not continue using the previous target.
}

// In the OpMode's finally/stop path:
limelight.stop();
```

Import `java.util.Optional`, `com.qualcomm.hardware.limelightvision.Limelight3A`,
and the targeting class when using it from another package. The example's setup,
loop, and stop sections belong in their respective OpMode lifecycle locations.

## Output contract

| Field | Meaning |
| --- | --- |
| `xDegrees` | Horizontal offset from optical axis, positive right |
| `yDegrees` | Vertical image-axis offset, positive up; `atan2(up, forward)` |
| `distanceMeters` | Straight-line distance to opening center |
| `rightMeters`, `upMeters`, `forwardMeters` | Opening's 3D camera-relative position |
| `planarDistanceMeters` | Range in camera right/forward plane; not ground range with pitched camera |
| `side` | Audience or scoring-table side (not an indication that scoring is allowed) |
| `rollDegrees` | SDK-style cluster roll used to reject inverted cells |
| `visibleTags`, `reprojectionErrorPixels` | Measurement quality indicators |
| `ageMillis`, `frameTimestamp` | Snapshot age and Limelight frame time; identical timestamps are not new observations |

Empty means no usable upright target in this frame: disconnected, invalid/stale
frame, wrong alliance/side, missing corners, inverted/edge-on cell, or poor fit.
Default limits are 200 ms total reported age, 3 pixels RMS reprojection error,
and 75 degrees absolute roll; the full constructor makes these configurable.
These defaults are initial tuning values, not experimentally validated thresholds.

An output is an **aiming candidate**, not permission to fire. A cell can be upright
while tipping; low reprojection error also does not eliminate planar pose ambiguity.
The firing controller must check settling on distinct frames, shooter readiness,
line of fire, and current measurement age. Recheck between shots. This class does
not infer the robot's field side from tag visibility; pass the side explicitly
when that distinction matters.

For a turret-mounted camera, angles remain relative to the camera at capture time;
account for turret motion since capture. For a fixed camera, transform the 3D point
using camera-to-robot calibration and turret angle. Both need the physical
camera-to-launch-point offset and a calibrated ballistic model; `yDegrees` is not
the required launcher elevation and distance alone is not a wheel-speed command.

## Validation

Build with JDK 17: `./gradlew :TeamCode:assembleDebug`.
Run host integration tests: `python3 TeamCode/tests/run_targeting_tests.py`.
The runner uses real SDK JSON parsing and desktop OpenCV 4.9 (Android SDK bundles
OpenCV 4.10); it downloads pinned test artifacts without changing app dependencies.
Synthetic projections cover aiming signs, opening offsets, single/multiple tags,
all four cells, side selection, inverted cells, stale/missing data, bad corners,
duplicate IDs and corner-index remapping. Hardware verification of calibration,
corner ordering, upright classification and latency remains necessary.

References: [SDK 12 source](https://repo.maven.apache.org/maven2/org/firstinspires/ftc/Vision/12.0.0/Vision-12.0.0-sources.jar)
(`AprilTagGameDatabase` geometry and `AprilTagProcessorImpl` pose convention),
[FIRST cluster guidance](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html),
[Limelight corner output](https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification).
