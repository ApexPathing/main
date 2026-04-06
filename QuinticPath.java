package followers.quintic;

import java.util.ArrayList;
import java.util.List;

import util.Pose;

/**
 * A full path made of chained QuinticSegments.
 * Handles global arc-length parameterization across all segments.
 *
 * Usage:
 *   QuinticPath path = new QuinticPath.Builder()
 *       .addPoint(0,  0,  0)
 *       .addPoint(24, 24, 90)
 *       .addPoint(48, 0,  0)
 *       .build();
 *
 * @author Xander Haemel 31616 404 Not Found
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class QuinticPath {

    private final List<QuinticSegment> segments;
    private final double[] segmentStartDistances;
    private final double totalLength;

    private QuinticPath(List<QuinticSegment> segments) {
        this.segments = segments;
        segmentStartDistances = new double[segments.size() + 1];
        segmentStartDistances[0] = 0;
        for (int i = 0; i < segments.size(); i++) {
            segmentStartDistances[i + 1] = segmentStartDistances[i] + segments.get(i).getLength();
        }
        totalLength = segmentStartDistances[segments.size()];
    }

    /** Total path length in inches */
    public double getLength() { return totalLength; }

    /** Get pose at a given arc distance along the entire path */
    public Pose getPoseAtDistance(double distance) {
        distance = Math.max(0, Math.min(distance, totalLength));
        int seg = getSegmentIndex(distance);
        double localDist = distance - segmentStartDistances[seg];
        double t = segments.get(seg).tFromDistance(localDist);
        return segments.get(seg).getPose(t);
    }

    /**
     * Find arc distance along path of the closest point to (px, py).
     */
    public double closestDistance(double px, double py) {
        double bestDist = Double.MAX_VALUE;
        double bestPathDist = 0;
        for (int i = 0; i < segments.size(); i++) {
            QuinticSegment seg = segments.get(i);
            double t = seg.closestT(px, py);
            double cx = seg.getX(t), cy = seg.getY(t);
            double d = Math.hypot(cx - px, cy - py);
            if (d < bestDist) {
                bestDist = d;
                bestPathDist = segmentStartDistances[i] + t * seg.getLength();
            }
        }
        return bestPathDist;
    }

    /** Get the lookahead pose: closest point + lookaheadDist ahead on path */
    public Pose getLookaheadPose(double px, double py, double lookaheadDist) {
        double target = Math.min(closestDistance(px, py) + lookaheadDist, totalLength);
        return getPoseAtDistance(target);
    }

    /** True if robot is within threshold inches of path end */
    public boolean isNearEnd(double px, double py, double threshold) {
        Pose end = getPoseAtDistance(totalLength);
        return Math.hypot(px - end.getX(), py - end.getY()) < threshold;
    }

    /** Progress as fraction [0, 1] */
    public double getProgress(double px, double py) {
        return closestDistance(px, py) / totalLength;
    }

    // ── Private ────────────────────────────────────────────────────────────────

    private int getSegmentIndex(double distance) {
        for (int i = segments.size() - 1; i >= 0; i--) {
            if (distance >= segmentStartDistances[i]) return i;
        }
        return 0;
    }

    // ── Builder ────────────────────────────────────────────────────────────────

    public static class Builder {
        private final List<Pose> poses = new ArrayList<>();
        private final List<Double> curvatures = new ArrayList<>();

        /** Add waypoint with heading in radians */
        public Builder addPose(Pose pose) {
            return addPose(pose, 0.0);
        }

        public Builder addPose(Pose pose, double curvature) {
            poses.add(pose);
            curvatures.add(curvature);
            return this;
        }

        /** Add waypoint with heading in degrees (convenience) */
        public Builder addPoint(double x, double y, double headingDegrees) {
            return addPose(new Pose(x, y, Math.toRadians(headingDegrees)));
        }

        public Builder addPoint(double x, double y, double headingDegrees, double curvature) {
            return addPose(new Pose(x, y, Math.toRadians(headingDegrees)), curvature);
        }

        public QuinticPath build() {
            if (poses.size() < 2) throw new IllegalStateException("Need at least 2 waypoints");
            List<QuinticSegment> segs = new ArrayList<>();
            for (int i = 0; i < poses.size() - 1; i++) {
                segs.add(new QuinticSegment(
                        poses.get(i), poses.get(i + 1),
                        curvatures.get(i), curvatures.get(i + 1)
                ));
            }
            return new QuinticPath(segs);
        }
    }
}