package geometry;

/**
 * A wrapper class that binds a mathematical parametric curve to physical properties.
 * <p>
 * This class handles the generation of a Look-Up Table (LUT) to precalculate
 * arc-length distances, enabling blisteringly fast O(1) distance lookups and
 * highly efficient closest-point projection using Newton-Raphson refinement.
 * Internally, all units are inches and radians.
 * <p>
 * Author: DrPixelCat
 */
public class PathSegment {
    private final ParametricSegment segment;
    private final double length;
    private final double POINTS_PER_INCH = 0.5;
    private final PathPoint[] LUTpoints;

    /**
     * Constructs a PathSegment and automatically generates its Look-Up Table (LUT).
     *
     * @param segment The underlying parametric geometry (e.g., Line, BSpline).
     */
    public PathSegment(ParametricSegment segment) {
        this.segment = segment;
        double length = calculateCoarseLength();

        int calculatedPoints = (int) (length * POINTS_PER_INCH);
        int numPoints = Math.max(2, calculatedPoints);

        this.LUTpoints = new PathPoint[numPoints];

        double distFromEnd = 0.0;
        Vector lastPoint = null;

        for (int i = numPoints - 1; i >= 0; i--) {
            double t = (double) i / (numPoints - 1);
            Vector location = segment.getPosition(t);

            if (lastPoint != null) {
                distFromEnd += lastPoint.minus(location).getMag().getIn();
            }
            lastPoint = location;
            LUTpoints[i] = new PathPoint(t, distFromEnd, location);
        }

        this.length = distFromEnd;
    }

    /**
     * Finds the closest parametric 't' value on the curve to the robot's physical location.
     * Uses a highly efficient two-step process: an O(N) coarse search through the LUT,
     * followed by a continuous Newton-Raphson root-finding refinement.
     *
     * @param location The current physical coordinate of the robot.
     * @return The parametric value 't' [0.0, 1.0] of the closest point on the curve.
     */
    public double getBestT(Vector location) {
        double bestT = 0;

        // IMPROVEMENT NOTE: 'coarseSelection' is assigned here and inside the loop,
        // but it is never actually used in the Newton-Raphson refinement below!
        // You can safely delete this variable to save a tiny bit of overhead.
        PathPoint coarseSelection = LUTpoints[0];
        double minDistSq = Double.MAX_VALUE;

        // Coarse search via LUT
        for (PathPoint point : LUTpoints) {
            double distSq = point.getLocation().minus(location).getMagSq().getIn();

            if (distSq < minDistSq) {
                minDistSq = distSq;
                bestT = point.getT();
                coarseSelection = point;
            }
        }

        // Newton-Raphson refinement
        for (int i = 0; i < 5; i++) {
            Vector b = segment.getPosition(bestT);
            Vector d1 = segment.getFirstDerivative(bestT);

            Vector diff = b.minus(location);
            double numerator = diff.dot(d1).getIn();

            // Optional Fast-Exit: If the distance vector is perfectly orthogonal to the tangent,
            // we have already found the exact closest point.
            if (Math.abs(numerator) < 1e-6 && bestT > 0.0 && bestT < 1.0) {
                break;
            }

            Vector d2 = segment.getSecondDerivative(bestT);
            double denominator = d1.dot(d1).plus(diff.dot(d2)).getIn();

            // Abort on singularity to prevent backward pushing
            if (denominator <= 0.0) {
                break;
            }

            double previousT = bestT;
            bestT = bestT - (numerator / denominator);
            bestT = Math.max(0.0, Math.min(1.0, bestT));

            if (Math.abs(bestT - previousT) < 1e-6) {
                break;
            }
        }

        // Update state at refined t
        return bestT;
    }

    /**
     * Retrieves the physical coordinate of the curve at a given 't'.
     *
     * @param t The parametric progression [0.0, 1.0].
     * @return The 2D position Vector.
     */
    public Vector getPosition(double t) {
        return segment.getPosition(t);
    }

    /**
     * Retrieves the first derivative (velocity) of the curve at a given 't'.
     *
     * @param t The parametric progression [0.0, 1.0].
     * @return The velocity Vector.
     */
    public Vector getFirstDerivative(double t) {
        return segment.getFirstDerivative(t);
    }

    /**
     * Retrieves the second derivative (acceleration) of the curve at a given 't'.
     *
     * @param t The parametric progression [0.0, 1.0].
     * @return The acceleration Vector.
     */
    public Vector getSecondDerivative(double t) {
        return segment.getSecondDerivative(t);
    }

    /**
     * Calculates the remaining physical distance to the end of the segment
     * using a blisteringly fast O(1) LUT index calculation.
     *
     * @param closestPointOnCurve The calculated physical position on the curve closest to the robot.
     * @param t                   The parametric 't' value that yielded closestPointOnCurve.
     * @return The remaining distance in inches.
     */
    public double getDistanceToEnd_in(Vector closestPointOnCurve, double t) {
        if (t >= 1.0) return 0.0;

        if (t <= 0.0) {
            double mag = closestPointOnCurve.minus(LUTpoints[0].getLocation()).getMag().getIn();
            return mag + LUTpoints[0].getDistanceToEnd_in();
        }

        int lastIndex = LUTpoints.length - 1;

        int nextIndex = (int) Math.ceil(t * lastIndex);

        nextIndex = Math.max(0, Math.min(nextIndex, lastIndex));

        PathPoint nextPoint = LUTpoints[nextIndex];

        double mag = closestPointOnCurve.minus(nextPoint.getLocation()).getMag().getIn();
        return mag + nextPoint.getDistanceToEnd_in();
    }

    // VERY approximate length calculation for Coarse Polyline Approximation

    /**
     * A highly optimized approximation of the segment's length used exclusively
     * to determine how many LUT points to allocate.
     *
     * @return An estimated arc-length in inches.
     */
    private double calculateCoarseLength() {
        final int SAMPLES = 8;
        double roughLength = 0.0;
        Vector prev = segment.getPosition(0.0);
        for (int i = 1; i <= SAMPLES; i++) {
            Vector curr = segment.getPosition((double) i / SAMPLES);
            roughLength += curr.minus(prev).getMag().getIn();
            prev = curr;
        }
        return roughLength;
    }

    /**
     * @return The high-accuracy calculated length of the segment in inches.
     */
    public double getLength_in() { return length; }

    /**
     * Calculates the instantaneous radius of curvature of a parametric curve at a specific point.
     * <p>
     * The radius of curvature is geometrically defined as the radius of the circular arc
     * which best approximates the curve at that point. It is computed using the magnitude
     * of the first derivative cubed, divided by the magnitude of the 2D cross product
     * of the first and second derivatives.
     *
     * @param firstDerivative  The first derivative vector (velocity/tangent) of the curve.
     * @param secondDerivative The second derivative vector (acceleration) of the curve.
     * @return The instantaneous radius of curvature. Returns Double.POSITIVE_INFINITY if the curve is perfectly straight.
     */
    public static double calculateRadiusOfCurvature(Vector firstDerivative, Vector secondDerivative) {
        // Use the Vector class's built-in 2D cross product utility
        double crossProductMag = firstDerivative.cross(secondDerivative).abs().getIn();

        // Safety Check: If the cross product is near zero, the derivatives are parallel,
        // meaning the path is a perfectly straight line with an infinite radius.
        if (crossProductMag < 1e-6) {
            return Double.POSITIVE_INFINITY;
        }

        // Use the Vector class's built-in magnitude utility to calculate ||r'||^3
        double velocityMag = firstDerivative.getMag().getIn();
        double numerator = Math.pow(velocityMag, 3);

        return numerator / crossProductMag;
    }

    /**
     * Retrieves the 2D principal unit normal vector to the curve at a given 't'.
     * <p>
     * The principal normal points strictly towards the center of curvature (the "inside"
     * of the curve). For maximum efficiency, this avoids trigonometric functions by swapping
     * coordinates, and uses the 2D cross product of velocity and acceleration to determine
     * the bend direction.
     *
     * @param firstDerivative The velocity vector of the segment at the closest point.
     * @param secondDerivative The acceleration vector of the segment at the closest point.
     * @return The principal unit normal Vector pointing toward the center of curvature. Returns (0, 0) if the cross product is zero.
     */
    public static Vector calculateArcNormal(Vector firstDerivative, Vector secondDerivative) {
        double vx = firstDerivative.getX(util.DistUnit.IN);
        double vy = firstDerivative.getY(util.DistUnit.IN);

        // .cross() returns a Dist object; .getIn() extracts its raw scalar value
        double cross = firstDerivative.cross(secondDerivative).getIn();

        // If the path is perfectly straight, there is no normal vector
        if (Math.abs(cross) < 1e-6) {
            return Vector.zero();
        }

        Vector normal;
        if (cross < 0) {
            // Bending right: swap X and Y, negate the new Y
            normal = Vector.of(vy, -vx, util.DistUnit.IN);
        } else {
            // Bending left: swap X and Y, negate the new X
            normal = Vector.of(-vy, vx, util.DistUnit.IN);
        }

        return normal.normalize();
    }
}