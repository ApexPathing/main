package followers.quintic;

import util.Pose;

/**
 * Represents a single quintic Hermite spline segment between two poses.
 * Provides C2 continuity (position, velocity, acceleration) across segments.
 *
 * Each axis (x, y) is independently parameterized by t in [0, 1].
 * p(t) = h00*p0 + h10*v0 + h20*a0 + h01*p1 + h11*v1 + h21*a1
 *
 * @author Xander Haemel 31616 404 Not Found
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class QuinticSegment {

    // Start conditions
    private final double x0, y0;   // position
    private final double vx0, vy0; // velocity (from heading)
    private final double ax0, ay0; // acceleration (from curvature)

    // End conditions
    private final double x1, y1;
    private final double vx1, vy1;
    private final double ax1, ay1;

    // Arc length table for t <-> distance mapping
    private static final int ARC_SAMPLES = 100;
    private final double[] arcLengthTable = new double[ARC_SAMPLES + 1];
    private double totalLength = 0;

    /**
     * @param start          start pose (x, y, heading in radians)
     * @param end            end pose (x, y, heading in radians)
     * @param startCurvature curvature at start (0 = straight)
     * @param endCurvature   curvature at end (0 = straight)
     */
    public QuinticSegment(Pose start, Pose end, double startCurvature, double endCurvature) {
        x0 = start.getX();
        y0 = start.getY();
        x1 = end.getX();
        y1 = end.getY();

        // Velocity magnitude scaled to chord length
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double tension = Math.hypot(dx, dy) * 1.2;

        vx0 = tension * Math.cos(start.getHeading());
        vy0 = tension * Math.sin(start.getHeading());
        vx1 = tension * Math.cos(end.getHeading());
        vy1 = tension * Math.sin(end.getHeading());

        // Acceleration from curvature: a = kappa * v^2 * normal direction
        double speed = tension;
        ax0 = startCurvature * speed * speed * (-Math.sin(start.getHeading()));
        ay0 = startCurvature * speed * speed * ( Math.cos(start.getHeading()));
        ax1 = endCurvature   * speed * speed * (-Math.sin(end.getHeading()));
        ay1 = endCurvature   * speed * speed * ( Math.cos(end.getHeading()));

        buildArcLengthTable();
    }

    // ── Basis polynomials ──────────────────────────────────────────────────────

    private double h00(double t) {
        double t3 = t*t*t, t4 = t3*t, t5 = t4*t;
        return 1 - 10*t3 + 15*t4 - 6*t5;
    }
    private double h10(double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
        return t - 6*t3 + 8*t4 - 3*t5;
    }
    private double h20(double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
        return 0.5*t2 - 1.5*t3 + 1.5*t4 - 0.5*t5;
    }
    private double h01(double t) {
        double t3 = t*t*t, t4 = t3*t, t5 = t4*t;
        return 10*t3 - 15*t4 + 6*t5;
    }
    private double h11(double t) {
        double t3 = t*t*t, t4 = t3*t, t5 = t4*t;
        return -4*t3 + 7*t4 - 3*t5;
    }
    private double h21(double t) {
        double t3 = t*t*t, t4 = t3*t, t5 = t4*t;
        return 0.5*t3 - t4 + 0.5*t5;
    }

    // ── Derivatives ────────────────────────────────────────────────────────────

    private double dh00(double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t;
        return -30*t2 + 60*t3 - 30*t4;
    }
    private double dh10(double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t;
        return 1 - 18*t2 + 32*t3 - 15*t4;
    }
    private double dh20(double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t;
        return t - 4.5*t2 + 6*t3 - 2.5*t4;
    }
    private double dh01(double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t;
        return 30*t2 - 60*t3 + 30*t4;
    }
    private double dh11(double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t;
        return -12*t2 + 28*t3 - 15*t4;
    }
    private double dh21(double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t;
        return 1.5*t2 - 4*t3 + 2.5*t4;
    }

    // ── Public evaluation ──────────────────────────────────────────────────────

    public double getX(double t) {
        return h00(t)*x0 + h10(t)*vx0 + h20(t)*ax0
                + h01(t)*x1 + h11(t)*vx1 + h21(t)*ax1;
    }

    public double getY(double t) {
        return h00(t)*y0 + h10(t)*vy0 + h20(t)*ay0
                + h01(t)*y1 + h11(t)*vy1 + h21(t)*ay1;
    }

    public double getHeading(double t) {
        double dx = dh00(t)*x0 + dh10(t)*vx0 + dh20(t)*ax0
                + dh01(t)*x1 + dh11(t)*vx1 + dh21(t)*ax1;
        double dy = dh00(t)*y0 + dh10(t)*vy0 + dh20(t)*ay0
                + dh01(t)*y1 + dh11(t)*vy1 + dh21(t)*ay1;
        return Math.atan2(dy, dx);
    }

    public Pose getPose(double t) {
        return new Pose(getX(t), getY(t), getHeading(t));
    }

    public double getLength() { return totalLength; }

    /**
     * Get curve parameter t from arc distance along this segment.
     */
    public double tFromDistance(double distance) {
        if (distance <= 0) return 0;
        if (distance >= totalLength) return 1;
        double step = totalLength / ARC_SAMPLES;
        int idx = Math.min((int)(distance / step), ARC_SAMPLES - 1);
        double lo = arcLengthTable[idx];
        double hi = arcLengthTable[idx + 1];
        double frac = (hi - lo > 1e-9) ? (distance - lo) / (hi - lo) : 0;
        return ((double) idx / ARC_SAMPLES) + frac * (1.0 / ARC_SAMPLES);
    }

    /**
     * Find the t value of the closest point on this segment to (px, py).
     */
    public double closestT(double px, double py) {
        double bestT = 0;
        double bestDist = Double.MAX_VALUE;
        for (int i = 0; i <= ARC_SAMPLES; i++) {
            double t = (double) i / ARC_SAMPLES;
            double d = dist2(px, py, t);
            if (d < bestDist) { bestDist = d; bestT = t; }
        }
        double lo = Math.max(0, bestT - 1.0 / ARC_SAMPLES);
        double hi = Math.min(1, bestT + 1.0 / ARC_SAMPLES);
        return goldenSearch(px, py, lo, hi);
    }

    // ── Private ────────────────────────────────────────────────────────────────

    private void buildArcLengthTable() {
        arcLengthTable[0] = 0;
        double prevX = getX(0), prevY = getY(0);
        for (int i = 1; i <= ARC_SAMPLES; i++) {
            double t = (double) i / ARC_SAMPLES;
            double cx = getX(t), cy = getY(t);
            arcLengthTable[i] = arcLengthTable[i-1] + Math.hypot(cx - prevX, cy - prevY);
            prevX = cx; prevY = cy;
        }
        totalLength = arcLengthTable[ARC_SAMPLES];
    }

    private double goldenSearch(double px, double py, double lo, double hi) {
        final double gr = (Math.sqrt(5) - 1) / 2;
        double c = hi - gr * (hi - lo);
        double d = lo + gr * (hi - lo);
        for (int i = 0; i < 20; i++) {
            if (dist2(px, py, c) < dist2(px, py, d)) hi = d;
            else lo = c;
            c = hi - gr * (hi - lo);
            d = lo + gr * (hi - lo);
        }
        return (lo + hi) / 2;
    }

    private double dist2(double px, double py, double t) {
        double dx = getX(t) - px, dy = getY(t) - py;
        return dx*dx + dy*dy;
    }
}