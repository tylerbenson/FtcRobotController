package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/**
 * Camera-relative upright-cell classification only. Odometry handles aiming/range.
 * Requires an upright camera and at least two visible tags of a cell. Validate
 * both hive positions over the intended viewing envelope before using as a firing gate.
 * Consistent orientation does not prove the hive is mechanically settled.
 * Single-threaded; caller owns pipeline selection and Limelight start/stop.
 */
public final class LimelightCellTargeting {
    public enum Alliance { RED, BLUE }
    public enum Side { AUDIENCE, SCORING_TABLE }
    public enum State { AUDIENCE_UP, SCORING_TABLE_UP, UNKNOWN }
    public enum Reason { CONFIRMED, CONFIRMING, NO_DATA, STALE, UNRELIABLE, CONFLICT }

    public static final class Observation {
        public final State state;
        public final Reason reason;
        public final int supportingTags, consecutiveFrames;
        public final double ageMillis;
        private Observation(State state, Reason reason, int tags, int frames, double age) {
            this.state = state; this.reason = reason; supportingTags = tags;
            consecutiveFrames = frames; ageMillis = age;
        }
        /** Vision gate only; caller must additionally check odometry aim and shooter readiness. */
        public boolean isUp(Side side) {
            Objects.requireNonNull(side, "side");
            return side == Side.AUDIENCE ? state == State.AUDIENCE_UP
                    : state == State.SCORING_TABLE_UP;
        }
    }

    private final Limelight3A limelight;
    private final Alliance alliance;
    private final int requiredFrames;
    private final long confirmationMillis, maxAgeMillis;
    private State pending = State.UNKNOWN;
    private Observation last = unknown(Reason.NO_DATA, 0);
    private int frames;
    private double frameTimestamp = Double.NaN;
    private long firstSeenMillis, pendingSinceMillis, lastCallMillis = Long.MIN_VALUE;

    /** No corner ordering, lens calibration or OpenCV pose solver is required. */
    public LimelightCellTargeting(Limelight3A limelight, Alliance alliance) {
        this(limelight, alliance, 3, 100, 200);
    }

    public LimelightCellTargeting(Limelight3A limelight, Alliance alliance,
                                 int requiredFrames, long confirmationMillis, long maxAgeMillis) {
        this.limelight = Objects.requireNonNull(limelight, "limelight");
        this.alliance = Objects.requireNonNull(alliance, "alliance");
        if (requiredFrames < 2 || confirmationMillis < 0 || maxAgeMillis <= 0) {
            throw new IllegalArgumentException("Invalid confirmation/freshness limits");
        }
        this.requiredFrames = requiredFrames;
        this.confirmationMillis = confirmationMillis;
        this.maxAgeMillis = maxAgeMillis;
    }

    public Observation update() {
        return evaluate(limelight.getLatestResult(), System.nanoTime() / 1_000_000);
    }

    /** nowMillis is monotonic time in milliseconds (injectable for deterministic tests). */
    public Observation evaluate(LLResult result, long nowMillis) {
        boolean gap = lastCallMillis != Long.MIN_VALUE &&
                (nowMillis < lastCallMillis || nowMillis - lastCallMillis > maxAgeMillis);
        lastCallMillis = nowMillis;
        if (gap) reset();
        if (result == null || !result.isValid()) return reject(Reason.NO_DATA, 0);
        double timestamp = result.getTimestamp();
        if (!Double.isFinite(timestamp) || timestamp <= 0) return reject(Reason.UNRELIABLE, 0);
        if (timestamp < frameTimestamp) reset(); // Camera restart: rebuild evidence.
        boolean newFrame = timestamp != frameTimestamp;
        if (newFrame) { frameTimestamp = timestamp; firstSeenMillis = nowMillis; }
        double age = Math.max(Math.max(0, result.getStaleness()), nowMillis - firstSeenMillis)
                + result.getCaptureLatency() + result.getTargetingLatency() + result.getParseLatency();
        if (!Double.isFinite(age) || age < 0 || age > maxAgeMillis) return reject(Reason.STALE, age);
        if (!newFrame) {
            return new Observation(last.state, last.reason, last.supportingTags, frames, age);
        }
        // SDK BIOBUZZ IDs increase left-to-right on each upright cluster.
        // Compare tag centers, which are independent of the order of their corners.
        double[][][] centers = new double[2][4][];
        int[] votes = new int[2], tags = new int[2];
        Set<Integer> seen = new HashSet<>();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            int cell = cellIndex(id);
            if (cell < 0) continue;
            if (!seen.add(id)) return reject(Reason.CONFLICT, age);
            double[] center = center(tag.getTargetCorners());
            if (center == null) return reject(Reason.UNRELIABLE, age);
            int firstId = alliance == Alliance.RED ? (cell == 0 ? 34 : 30) : (cell == 0 ? 38 : 42);
            centers[cell][id - firstId] = center;
            tags[cell]++;
        }
        for (int cell = 0; cell < 2; cell++) {
            for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) {
                if (centers[cell][i] == null || centers[cell][j] == null) continue;
                double dx = centers[cell][j][0] - centers[cell][i][0];
                double dy = centers[cell][j][1] - centers[cell][i][1];
                double length = Math.hypot(dx, dy);
                // Reject near-vertical ordering and nearly overlapping detections.
                if (length < 4 || Math.abs(dx) / length <= .5) return reject(Reason.UNRELIABLE, age);
                votes[cell] |= dx > 0 ? 1 : 2;
            }
        }
        if (votes[0] == 3 || votes[1] == 3 || (votes[0] == 1 && votes[1] == 1)) {
            return reject(Reason.CONFLICT, age);
        }
        int upright = votes[0] == 1 ? 0 : votes[1] == 1 ? 1 : -1;
        // Require directly observed upright evidence; never infer an unseen opposite cell.
        if (upright < 0) return reject(Reason.NO_DATA, age);
        State candidate = upright == 0 ? State.AUDIENCE_UP : State.SCORING_TABLE_UP;
        if (candidate != pending) {
            pending = candidate; frames = 0; pendingSinceMillis = nowMillis;
        }
        frames = Math.min(frames + 1, requiredFrames);
        boolean confirmed = frames >= requiredFrames && nowMillis - pendingSinceMillis >= confirmationMillis;
        last = new Observation(confirmed ? candidate : State.UNKNOWN,
                confirmed ? Reason.CONFIRMED : Reason.CONFIRMING, tags[upright], frames, age);
        return last;
    }

    public void reset() {
        pending = State.UNKNOWN; frames = 0; frameTimestamp = Double.NaN;
        last = unknown(Reason.NO_DATA, 0);
    }

    private Observation reject(Reason reason, double age) {
        pending = State.UNKNOWN; frames = 0;
        // Keep frameTimestamp: repeated polls of a rejected frame cannot rebuild confirmation.
        last = unknown(reason, age);
        return last;
    }

    private static Observation unknown(Reason reason, double age) {
        return new Observation(State.UNKNOWN, reason, 0, 0, age);
    }

    private int cellIndex(int id) {
        if (alliance == Alliance.RED) {
            if (id >= 34 && id <= 37) return 0;
            if (id >= 30 && id <= 33) return 1;
        } else {
            if (id >= 38 && id <= 41) return 0;
            if (id >= 42 && id <= 45) return 1;
        }
        return -1;
    }

    /** Mean of four tag corners; no semantic corner ordering is assumed. */
    private static double[] center(List<List<Double>> corners) {
        if (corners == null || corners.size() != 4) return null;
        double x = 0, y = 0;
        for (int i = 0; i < 4; i++) {
            List<Double> c = corners.get(i);
            if (c == null || c.size() != 2 || c.get(0) == null || c.get(1) == null
                    || !Double.isFinite(c.get(0)) || !Double.isFinite(c.get(1))) return null;
            for (int j = 0; j < i; j++) {
                List<Double> other = corners.get(j);
                if (Math.hypot(c.get(0)-other.get(0), c.get(1)-other.get(1)) < 1) return null;
            }
            x += c.get(0) / 4; y += c.get(1) / 4;
        }
        return Double.isFinite(x) && Double.isFinite(y) ? new double[]{x, y} : null;
    }
}
