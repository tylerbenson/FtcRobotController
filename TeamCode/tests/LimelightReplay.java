package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Locale;
import java.util.Optional;

import static org.firstinspires.ftc.teamcode.LimelightCellTargeting.*;

/** Runs captured JSON through the actual production targeting class without hardware. */
public final class LimelightReplay {
    private static final class RecordedResult extends LLResult {
        private final long staleness;
        RecordedResult(JSONObject payload, long staleness) throws Exception {
            super(payload);
            this.staleness = staleness;
        }
        @Override public long getStaleness() { return staleness; }
        // Parsing on the desktop isn't historical Control Hub parsing latency.
        @Override public double getParseLatency() { return 0; }
    }

    public static void main(String[] args) throws Exception {
        Path session = Paths.get(args[0]);
        JSONObject metadata = new JSONObject(Files.readString(session.resolve("session.json")));
        if (metadata.getInt("schema_version") != 1) {
            throw new IllegalArgumentException("Unsupported session schema");
        }
        JSONObject config = args[1].isEmpty() ? metadata.optJSONObject("calibration")
                : new JSONObject(Files.readString(Paths.get(args[1])));
        if (config == null) throw new IllegalArgumentException(
                "Replay needs --calibration FILE or calibration embedded at recording time");
        JSONArray coefficients = config.getJSONArray("distortion");
        double[] distortion = new double[coefficients.length()];
        for (int i = 0; i < distortion.length; i++) distortion[i] = coefficients.getDouble(i);
        JSONArray order = config.getJSONArray("corner_order");
        int[] corners = new int[order.length()];
        for (int i = 0; i < corners.length; i++) corners[i] = order.getInt(i);
        Calibration calibration = new Calibration(config.getDouble("fx"), config.getDouble("fy"),
                config.getDouble("cx"), config.getDouble("cy"), distortion, corners);
        // No start() call: this object is only used to satisfy the production constructor.
        Limelight3A camera = new Limelight3A(null, "offline-replay", InetAddress.getLoopbackAddress());
        Alliance alliance = Alliance.valueOf(metadata.getString("alliance").toUpperCase(Locale.ROOT));
        LimelightCellTargeting targeting = new LimelightCellTargeting(camera, alliance, calibration);
        String sideName = metadata.optString("side", "either");
        Side side = sideName.equals("either") ? null
                : Side.valueOf(sideName.replace('-', '_').toUpperCase(Locale.ROOT));
        Path outputPath = args[2].isEmpty() ? session.resolve("targeting.csv") : Paths.get(args[2]);
        // Avoid accidentally overwriting a source recording or calibration file.
        if (!outputPath.getFileName().toString().endsWith(".csv")) {
            throw new IllegalArgumentException("Replay output must end in .csv");
        }
        int count = 0, accepted = 0;
        try (BufferedReader input = Files.newBufferedReader(session.resolve("frames.jsonl"));
             BufferedWriter output = Files.newBufferedWriter(outputPath, StandardCharsets.UTF_8)) {
            output.write("seq,elapsed_ms,status,duplicate,side,x_degrees,y_degrees,distance_m,"
                    + "right_m,up_m,forward_m,roll_degrees,visible_tags,reprojection_error_px,"
                    + "age_ms,expected_distance_m,distance_error_m\n");
            String line;
            while ((line = input.readLine()) != null) {
                JSONObject row;
                try { row = new JSONObject(line); }
                catch (RuntimeException error) {
                    throw new IllegalArgumentException("Invalid recording row " + (count + 1)
                            + "; repair any interrupted final row before replay", error);
                }
                count++;
                Optional<Target> target = Optional.empty();
                String status = "no_target";
                if (row.has("error")) {
                    status = "read_error";
                } else {
                    try {
                        JSONObject response = row.getJSONObject("response");
                        JSONObject payload = response.has("Results")
                                ? response.getJSONObject("Results") : response;
                        // The SDK reads legacy milliseconds; newer firmware may only send ts_us.
                        if (!payload.has("ts") && payload.has("ts_us")) {
                            payload.put("ts", payload.getDouble("ts_us") / 1000.0);
                        }
                        target = targeting.evaluate(new RecordedResult(payload,
                                Math.max(0, Math.round(row.optDouble("observed_staleness_ms", 0)))), side);
                    } catch (RuntimeException error) {
                        status = "invalid_payload";
                    }
                }
                Object[] values = new Object[17];
                values[0] = row.getInt("seq");
                values[1] = row.getDouble("elapsed_ms");
                values[2] = status;
                values[3] = row.isNull("duplicate") ? "" : row.getBoolean("duplicate");
                if (target.isPresent()) {
                    accepted++;
                    Target t = target.get();
                    values[2] = "candidate";
                    values[4] = t.side;
                    values[5] = t.xDegrees; values[6] = t.yDegrees;
                    values[7] = t.distanceMeters;
                    values[8] = t.rightMeters; values[9] = t.upMeters; values[10] = t.forwardMeters;
                    values[11] = t.rollDegrees; values[12] = t.visibleTags;
                    values[13] = t.reprojectionErrorPixels; values[14] = t.ageMillis;
                    if (!metadata.isNull("expected_distance_m")) {
                        double expected = metadata.getDouble("expected_distance_m");
                        values[15] = expected; values[16] = t.distanceMeters - expected;
                    }
                }
                for (int i = 0; i < values.length; i++) {
                    if (i > 0) output.write(',');
                    if (values[i] != null) output.write(values[i].toString());
                }
                output.write('\n');
            }
        }
        System.out.println("Replayed " + count + " polls; " + accepted + " aiming candidates. " + outputPath);
    }
}
