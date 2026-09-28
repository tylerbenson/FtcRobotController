package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.json.JSONArray;
import org.json.JSONObject;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import java.net.InetAddress;

import static org.firstinspires.ftc.teamcode.LimelightCellTargeting.*;

/** Host integration tests: real SDK JSON parser and native OpenCV, no robot/network connection. */
public final class LimelightCellTargetingTest {
    private static int checks;
    private static final class Frame extends LLResult {
        long stale;
        Frame(JSONArray tags) throws Exception {
            super(new JSONObject().put("v", 1).put("ts", 1234).put("Fiducial", tags));
        }
        @Override public long getStaleness() { return stale; }
    }
    private static final class Camera extends Limelight3A {
        LLResult frame;
        Camera() throws Exception { super(null, "test", InetAddress.getLoopbackAddress()); }
        @Override public LLResult getLatestResult() { return frame; }
    }
    private static final Calibration CALIBRATION = new Calibration(
            700, 710, 640, 360, new double[0], new int[]{0, 1, 2, 3});

    public static void main(String[] args) throws Exception {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        Camera camera = new Camera();
        LimelightCellTargeting red = new LimelightCellTargeting(camera, Alliance.RED, CALIBRATION);
        LimelightCellTargeting blue = new LimelightCellTargeting(camera, Alliance.BLUE, CALIBRATION);

        camera.frame = new Frame(tags(34, 4, .4, -.3, 2, .15, -.2, .1));
        Target t = red.update().orElseThrow(AssertionError::new);
        close(t.rightMeters, .4, "right");
        close(t.upMeters, .3, "up");
        close(t.forwardMeters, 2, "forward");
        close(t.distanceMeters, Math.sqrt(4.25), "slant distance");
        close(t.xDegrees, Math.toDegrees(Math.atan2(.4, 2)), "horizontal angle");
        close(t.yDegrees, Math.toDegrees(Math.atan2(.3, 2)), "vertical angle");
        check(t.side == Side.AUDIENCE && t.visibleTags == 4, "identity and tag count");
        check(!red.update(Side.SCORING_TABLE).isPresent(), "side filtering");
        check(!blue.update().isPresent(), "alliance filtering");

        camera.frame = new Frame(tags(30, 1, -.2, .15, 1.5, .25, .1, -.15));
        t = red.update().orElseThrow(AssertionError::new);
        close(t.rightMeters, -.2, "single tag right");
        close(t.upMeters, -.15, "single tag up");
        close(t.forwardMeters, 1.5, "single tag range");
        check(t.visibleTags == 1 && t.side == Side.SCORING_TABLE, "single tag identity");

        camera.frame = new Frame(tags(42, 3, 0, -.2, 2.5, 0, 0, 0));
        check(blue.update(Side.SCORING_TABLE).isPresent(), "blue scoring mapping");
        camera.frame = new Frame(tags(38, 2, 0, -.2, 2.5, 0, 0, 0));
        check(blue.update(Side.AUDIENCE).isPresent(), "blue audience mapping");

        camera.frame = new Frame(tags(34, 4, 0, -.3, 2, 0, 0, Math.PI));
        check(!red.update().isPresent(), "inverted cell rejected");
        camera.frame = new Frame(tags(34, 4, 0, -.3, 2, 0, 0, Math.toRadians(85)));
        check(!red.update().isPresent(), "roll margin rejected");

        JSONArray both = tags(30, 4, 0, -.3, 2, 0, 0, Math.PI);
        append(both, tags(34, 4, .3, -.3, 3, 0, 0, 0));
        camera.frame = new Frame(both);
        check(red.update().get().side == Side.AUDIENCE, "visible inverted near cell ignored");
        both = tags(30, 4, 0, -.3, 2, 0, 0, 0);
        append(both, tags(34, 4, .3, -.3, 3, 0, 0, 0));
        camera.frame = new Frame(both);
        check(red.update().get().side == Side.SCORING_TABLE, "nearest candidate");
        check(red.update(Side.AUDIENCE).get().side == Side.AUDIENCE, "explicit side wins");

        Frame old = new Frame(tags(34, 4, 0, 0, 2, 0, 0, 0));
        old.stale = 500;
        camera.frame = old;
        check(!red.update().isPresent(), "stale frame rejected");
        camera.frame = null;
        check(!red.update().isPresent(), "no cached target after disconnect");
        camera.frame = new Frame(new JSONArray());
        check(!red.update().isPresent(), "empty frame rejected");
        camera.frame = new Frame(new JSONArray().put(new JSONObject().put("fID", 34)));
        check(!red.update().isPresent(), "missing corners rejected");

        JSONArray bad = tags(34, 4, 0, 0, 2, 0, 0, 0);
        bad.getJSONObject(0).getJSONArray("pts").getJSONArray(0).put(0, 100.0);
        camera.frame = new Frame(bad);
        check(!red.update().isPresent(), "inconsistent corners rejected");

        JSONArray duplicate = tags(34, 1, 0, 0, 2, 0, 0, 0);
        duplicate.put(duplicate.getJSONObject(0));
        camera.frame = new Frame(duplicate);
        check(red.update().get().visibleTags == 1, "duplicates not double counted");
        // The mapping must preserve decoded tag orientation, even with a different list order.
        JSONArray reordered = tags(34, 4, .2, -.3, 2, 0, 0, 0);
        for (int i = 0; i < reordered.length(); i++) {
            JSONArray pts = reordered.getJSONObject(i).getJSONArray("pts");
            reordered.getJSONObject(i).put("pts", new JSONArray()
                    .put(pts.get(2)).put(pts.get(0)).put(pts.get(3)).put(pts.get(1)));
        }
        LimelightCellTargeting remapped = new LimelightCellTargeting(camera, Alliance.RED,
                new Calibration(700, 710, 640, 360, new double[0], new int[]{1, 3, 0, 2}));
        camera.frame = new Frame(reordered);
        close(remapped.update().get().rightMeters, .2, "corner remapping");
        if (args.length == 1) writeReplayFixture(java.nio.file.Paths.get(args[0]));
        System.out.println("PASS: " + checks + " targeting checks");
    }

    /** Optional deterministic recording fixture for end-to-end desktop replay tests. */
    private static void writeReplayFixture(java.nio.file.Path directory) throws Exception {
        java.nio.file.Files.createDirectories(directory);
        JSONObject calibration = new JSONObject().put("fx", 700).put("fy", 710)
                .put("cx", 640).put("cy", 360).put("distortion", new JSONArray())
                .put("corner_order", new JSONArray(new int[]{0, 1, 2, 3}));
        JSONObject metadata = new JSONObject().put("schema_version", 1).put("alliance", "red")
                .put("side", "audience").put("calibration", calibration)
                .put("expected_distance_m", Math.sqrt(4.25));
        java.nio.file.Files.writeString(directory.resolve("session.json"), metadata.toString());
        try (java.io.BufferedWriter writer = java.nio.file.Files.newBufferedWriter(directory.resolve("frames.jsonl"))) {
            for (int i = 0; i < 5; i++) {
                JSONObject payload = new JSONObject().put("v", 1).put("ts", i)
                        .put("Fiducial", tags(i == 2 ? 38 : 34, 4, .4, -.3, 2,
                                0, 0, i == 1 ? Math.PI : 0));
                JSONObject row = new JSONObject().put("seq", i).put("elapsed_ms", i * 100)
                        .put("duplicate", i == 3).put("observed_staleness_ms", i == 3 ? 500 : 0);
                if (i == 4) row.put("error", "synthetic disconnect");
                else row.put("response", i == 0 ? new JSONObject().put("Results", payload) : payload);
                writer.write(row.toString());
                writer.newLine();
            }
        }
    }

    private static void append(JSONArray to, JSONArray from) {
        for (int i = 0; i < from.length(); i++) to.put(from.get(i));
    }

    /** Render known cluster geometry into a synthetic pinhole image. */
    private static JSONArray tags(int firstId, int count, double x, double y, double z,
                                  double rx, double ry, double rz) throws Exception {
        JSONArray tags = new JSONArray();
        Mat camera = Mat.eye(3, 3, CvType.CV_64F);
        camera.put(0, 0, 700); camera.put(1, 1, 710);
        camera.put(0, 2, 640); camera.put(1, 2, 360);
        Mat r = new Mat(3, 1, CvType.CV_64F), t = new Mat(3, 1, CvType.CV_64F);
        r.put(0, 0, rx, ry, rz); t.put(0, 0, x, y, z);
        MatOfDouble distortion = new MatOfDouble();
        try {
            double[] centers = {-6.5, -2.75, 2.75, 6.5};
            for (int i = 0; i < count; i++) {
                // Fixture explicitly encodes SDK dimensions rather than calling production geometry.
                double left = (centers[i] - 1.625) * .0254;
                double right = (centers[i] + 1.625) * .0254;
                double bottom = 8.8124 * .0254, top = 5.5624 * .0254, depth = -5.622 * .0254;
                MatOfPoint3f points = new MatOfPoint3f(new Point3(left, bottom, depth),
                        new Point3(right, bottom, depth), new Point3(right, top, depth),
                        new Point3(left, top, depth));
                MatOfPoint2f pixels = new MatOfPoint2f();
                try {
                    Calib3d.projectPoints(points, r, t, camera, distortion, pixels);
                    JSONArray corners = new JSONArray();
                    for (Point p : pixels.toArray()) corners.put(new JSONArray().put(p.x).put(p.y));
                    tags.put(new JSONObject().put("fID", firstId + i).put("pts", corners));
                } finally { points.release(); pixels.release(); }
            }
        } finally { camera.release(); r.release(); t.release(); distortion.release(); }
        return tags;
    }

    private static void close(double actual, double expected, String label) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < .001,
                label + ": expected " + expected + ", got " + actual);
    }
    private static void check(boolean success, String label) {
        if (!success) throw new AssertionError(label);
        checks++;
    }
}
