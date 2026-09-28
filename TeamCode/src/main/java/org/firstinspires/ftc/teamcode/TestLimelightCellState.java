package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Live, vision-only cell-state test. Requires only a configured Limelight 3A. */
@TeleOp(name = "Test: Limelight Cell State", group = "Test")
public class TestLimelightCellState extends LinearOpMode {
    // Configure this slot for AprilTag 36h11, IDs 30-45, with corner output enabled.
    private static final int APRILTAG_PIPELINE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightCellTargeting red = new LimelightCellTargeting(
                limelight, LimelightCellTargeting.Alliance.RED);
        LimelightCellTargeting blue = new LimelightCellTargeting(
                limelight, LimelightCellTargeting.Alliance.BLUE);
        telemetry.setMsTransmissionInterval(100);

        try {
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            limelight.start();
            // Preview during INIT, then continue updating until STOP.
            while (!isStopRequested() && (opModeInInit() || opModeIsActive())) {
                // Both classifiers evaluate the same camera frame and monotonic time.
                LLResult result = limelight.getLatestResult();
                long nowMillis = System.nanoTime() / 1_000_000;
                LimelightCellTargeting.Observation redState = red.evaluate(result, nowMillis);
                LimelightCellTargeting.Observation blueState = blue.evaluate(result, nowMillis);

                telemetry.addData("Mode", isStarted() ? "RUNNING" : "INIT preview - press Play");
                telemetry.addData("Pipeline requested", APRILTAG_PIPELINE);
                showState("RED", redState);
                showState("BLUE", blueState);
                telemetry.addData("IDs in latest result", tagIds(result));
                telemetry.addData("Result", result == null ? "No camera result"
                        : result.isValid() ? "Valid (see age/reason above)" : "No valid detections");
                telemetry.addLine("Move camera: each alliance updates independently.");
                telemetry.addLine("Needs 2 tags per cell; UNKNOWN = not confirmed in this view.");
                telemetry.update();
                sleep(20);
            }
        } finally {
            limelight.stop();
        }
    }

    private void showState(String alliance, LimelightCellTargeting.Observation state) {
        telemetry.addData(alliance + " cell UP", state.state);
        telemetry.addData(alliance + " evidence", "%s | tags %d | frames %d | age %.0f ms",
                state.reason, state.supportingTags, state.consecutiveFrames, state.ageMillis);
    }

    private static String tagIds(LLResult result) {
        if (result == null || !result.isValid()) return "none";
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (ids.length() > 0) ids.append(", ");
            ids.append(tag.getFiducialId());
        }
        return ids.length() == 0 ? "none" : ids.toString();
    }
}
