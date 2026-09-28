package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.CvType;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

/**
 * BIOBUZZ cell opening measurements relative to the camera's optical center.
 * Call update() from one OpMode thread. The caller selects/starts/stops the Limelight pipeline.
 * Requires AprilTag corner output and calibration for the exact pipeline resolution/crop.
 * Does not use Limelight botpose, individual tag poses, or crosshair offsets.
 *
 * Camera must be mounted upright (no roll), looking generally toward the launcher target.
 * The upright test is a camera-relative heuristic, NOT a settled-cell or fire authorization.
 * Camera/shooter offset, turret encoder, latency compensation and ballistics belong downstream.
 */
public final class LimelightCellTargeting {
    public enum Alliance { RED, BLUE }
    public enum Side { AUDIENCE, SCORING_TABLE }

    /** Immutable measurements; angles are degrees, distances are meters. */
    public static final class Target {
        public final Side side;
        /** Positive right/up, measured from the camera optical axis (not a UI crosshair). */
        public final double xDegrees, yDegrees;
        /** Opening position in camera axes: right, up, forward. */
        public final double rightMeters, upMeters, forwardMeters;
        /** Straight-line distance to the opening; not horizontal ground distance. */
        public final double distanceMeters;
        /** Range in the camera's right/forward plane; ground range only for a level camera. */
        public final double planarDistanceMeters;
        public final double rollDegrees;
        public final int visibleTags;
        public final double reprojectionErrorPixels;
        /** Age including reported capture/processing/parsing latency, measured at update(). */
        public final double ageMillis;
        /** Limelight frame timestamp, useful for avoiding repeated-frame stability counts. */
        public final double frameTimestamp;

        private Target(Side side, double right, double up, double forward, double roll,
                       int visibleTags, double error, double age, double timestamp) {
            this.side = side;
            rightMeters = right;
            upMeters = up;
            forwardMeters = forward;
            xDegrees = Math.toDegrees(Math.atan2(right, forward));
            yDegrees = Math.toDegrees(Math.atan2(up, forward));
            planarDistanceMeters = Math.hypot(right, forward);
            distanceMeters = Math.hypot(planarDistanceMeters, up);
            rollDegrees = roll;
            this.visibleTags = visibleTags;
            reprojectionErrorPixels = error;
            ageMillis = age;
            frameTimestamp = timestamp;
        }
    }

    /** Calibration and rejection limits. No fabricated camera calibration defaults. */
    public static final class Calibration {
        final double fx, fy, cx, cy;
        final double[] distortion;
        final int[] cornerOrder;

        /**
         * Pixel coordinates must be absolute, top-left origin, x right/y down.
         * cornerOrder maps SDK corners to Limelight list indices: bottom-left, bottom-right,
         * top-right, top-left of the DECODED upright tag, not sorted screen positions.
         * Verify this order with a real tag; sorting corners by screen position erases inversion.
         * Distortion is OpenCV [k1,k2,p1,p2,k3,...]; empty only for undistorted coordinates.
         */
        public Calibration(double fx, double fy, double cx, double cy,
                           double[] distortion, int[] cornerOrder) {
            if (!finite(fx, fy, cx, cy) || fx <= 0 || fy <= 0) {
                throw new IllegalArgumentException("Invalid camera intrinsics");
            }
            Objects.requireNonNull(distortion, "distortion");
            Objects.requireNonNull(cornerOrder, "cornerOrder");
            int n = distortion.length;
            if (!(n == 0 || n == 4 || n == 5 || n == 8 || n == 12 || n == 14)
                    || !finite(distortion)) {
                throw new IllegalArgumentException("Invalid OpenCV distortion coefficients");
            }
            Set<Integer> indices = new HashSet<>();
            for (int index : cornerOrder) indices.add(index);
            if (cornerOrder.length != 4 || indices.size() != 4
                    || !indices.contains(0) || !indices.contains(1)
                    || !indices.contains(2) || !indices.contains(3)) {
                throw new IllegalArgumentException("cornerOrder must be a permutation of 0..3");
            }
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            this.distortion = distortion.clone();
            this.cornerOrder = cornerOrder.clone();
        }
    }

    private final Limelight3A limelight;
    private final Alliance alliance;
    private final Calibration calibration;
    private final double maxAgeMillis, maxErrorPixels, maxAbsRollDegrees;

    public LimelightCellTargeting(Limelight3A limelight, Alliance alliance,
                                 Calibration calibration) {
        this(limelight, alliance, calibration, 200, 3, 75);
    }

    /** Rejection limits are starting points to validate on the actual robot. */
    public LimelightCellTargeting(Limelight3A limelight, Alliance alliance,
                                 Calibration calibration, double maxAgeMillis,
                                 double maxErrorPixels, double maxAbsRollDegrees) {
        this.limelight = Objects.requireNonNull(limelight, "limelight");
        this.alliance = Objects.requireNonNull(alliance, "alliance");
        this.calibration = Objects.requireNonNull(calibration, "calibration");
        if (!finite(maxAgeMillis, maxErrorPixels, maxAbsRollDegrees)
                || maxAgeMillis <= 0 || maxErrorPixels <= 0
                || maxAbsRollDegrees <= 0 || maxAbsRollDegrees >= 90) {
            throw new IllegalArgumentException("Invalid targeting rejection limits");
        }
        this.maxAgeMillis = maxAgeMillis;
        this.maxErrorPixels = maxErrorPixels;
        this.maxAbsRollDegrees = maxAbsRollDegrees;
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    /** Select the nearest upright, valid cell of this alliance. No previous target is retained. */
    public Optional<Target> update() {
        return update(null);
    }

    /** Restrict to our current side when known; null permits either side. */
    public Optional<Target> update(Side side) {
        return evaluate(limelight.getLatestResult(), side);
    }

    /** Also accepts a shared snapshot so multiple consumers can use the same Limelight frame. */
    public Optional<Target> evaluate(LLResult result, Side side) {
        if (result == null || !result.isValid()) return Optional.empty();
        double age = Math.max(0, result.getStaleness()) + result.getCaptureLatency()
                + result.getTargetingLatency() + result.getParseLatency();
        if (!finite(age, result.getTimestamp()) || age < 0 || age > maxAgeMillis) {
            return Optional.empty();
        }
        Target best = null;
        for (Side candidate : Side.values()) {
            if (side != null && side != candidate) continue;
            Target target = solve(result, candidate, age);
            if (target != null && (best == null || target.distanceMeters < best.distanceMeters)) {
                best = target;
            }
        }
        return Optional.ofNullable(best);
    }

    private Target solve(LLResult result, Side side, double age) {
        int firstId = alliance == Alliance.RED
                ? (side == Side.SCORING_TABLE ? 30 : 34)
                : (side == Side.SCORING_TABLE ? 42 : 38);
        List<Point> imagePoints = new ArrayList<>();
        List<Point3> objectPoints = new ArrayList<>();
        Set<Integer> ids = new HashSet<>();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int member = tag.getFiducialId() - firstId;
            if (member < 0 || member > 3 || ids.contains(tag.getFiducialId())) continue;
            List<List<Double>> corners = tag.getTargetCorners();
            if (!validCorners(corners)) continue;
            ids.add(tag.getFiducialId());
            for (int corner = 0; corner < 4; corner++) {
                List<Double> p = corners.get(calibration.cornerOrder[corner]);
                imagePoints.add(new Point(p.get(0), p.get(1)));
                objectPoints.add(clusterCorner(member, corner));
            }
        }
        if (ids.isEmpty()) return null;

        Mat camera = Mat.eye(3, 3, CvType.CV_64F);
        MatOfDouble distortion = new MatOfDouble(calibration.distortion);
        MatOfPoint2f image = new MatOfPoint2f();
        MatOfPoint3f object = new MatOfPoint3f();
        Mat rotationVector = new Mat(), translation = new Mat(), rotation = new Mat();
        MatOfPoint2f projected = new MatOfPoint2f();
        try {
            camera.put(0, 0, calibration.fx);
            camera.put(1, 1, calibration.fy);
            camera.put(0, 2, calibration.cx);
            camera.put(1, 2, calibration.cy);
            image.fromList(imagePoints);
            object.fromList(objectPoints);
            if (!Calib3d.solvePnP(object, image, camera, distortion, rotationVector,
                    translation, false, Calib3d.SOLVEPNP_ITERATIVE)) return null;
            Calib3d.Rodrigues(rotationVector, rotation);
            double[] r = new double[9], t = new double[3];
            rotation.get(0, 0, r);
            translation.get(0, 0, t);
            if (!finite(r) || !finite(t) || t[2] <= 0) return null;
            for (Point3 p : objectPoints) {
                if (r[6] * p.x + r[7] * p.y + r[8] * p.z + t[2] <= 0) return null;
            }
            // Same roll as SDK raw-pose INTRINSIC YXZ decomposition away from gimbal lock.
            if (Math.hypot(r[3], r[4]) < 1e-6) return null;
            double roll = Math.toDegrees(Math.atan2(r[3], r[4]));
            if (Math.abs(roll) > maxAbsRollDegrees) return null;
            Calib3d.projectPoints(object, rotationVector, translation, camera, distortion, projected);
            Point[] projectedPoints = projected.toArray();
            double squaredError = 0;
            for (int i = 0; i < projectedPoints.length; i++) {
                double dx = projectedPoints[i].x - imagePoints.get(i).x;
                double dy = projectedPoints[i].y - imagePoints.get(i).y;
                squaredError += dx * dx + dy * dy;
            }
            double error = Math.sqrt(squaredError / imagePoints.size());
            if (!finite(error) || error > maxErrorPixels) return null;
            return new Target(side, t[0], -t[1], t[2], roll, ids.size(), error,
                    age, result.getTimestamp());
        } catch (CvException invalidGeometry) {
            return null;
        } finally {
            camera.release();
            distortion.release();
            image.release();
            object.release();
            rotationVector.release();
            translation.release();
            rotation.release();
            projected.release();
        }
    }

    /** Geometry from SDK 12.0 AprilTagGameDatabase.getBioBuzzTagLibrary(), in meters. */
    static Point3 clusterCorner(int member, int corner) {
        double[] centers = {-6.50, -2.75, 2.75, 6.50};
        double half = 3.25 / 2;
        return new Point3((centers[member] + (corner == 0 || corner == 3 ? -half : half)) * .0254,
                (7.1874 + (corner < 2 ? half : -half)) * .0254, -5.622 * .0254);
    }

    private boolean validCorners(List<List<Double>> corners) {
        if (corners == null || corners.size() != 4) return false;
        for (List<Double> p : corners) {
            if (p == null || p.size() != 2 || p.get(0) == null || p.get(1) == null
                    || !finite(p.get(0), p.get(1))) return false;
        }
        // Reject missing/degenerate polygons before invoking the solver.
        double areaTwice = 0;
        for (int i = 0; i < 4; i++) {
            List<Double> p = corners.get(calibration.cornerOrder[i]);
            List<Double> q = corners.get(calibration.cornerOrder[(i + 1) % 4]);
            areaTwice += p.get(0) * q.get(1) - q.get(0) * p.get(1);
        }
        return Math.abs(areaTwice) > 1;
    }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
}
