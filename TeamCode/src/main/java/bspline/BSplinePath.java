package bspline;
import java.util.ArrayList;
import java.util.List;

/**
 * Math class for B splines
 * Calculates B spline curves
 * Usage: List<double[]> points = List.of(
 *          new double[]{0, 0},
 *          new double[]{12, 24},
 *          new double[]{36, 67},
 *          new double[]{60, 24},
 *          new double[]{72, 6.7}
 *        );
 *        BSplinePath path = new BSplinePath(points, 3) (Degree 3 for cubic)
 * Uses De Boor's recursive algorithm :)
 * @author Sohum Arora - 22985 Paraducks (@paradoxical-duck)
 */
public class BSplinePath {
    int degree; // use 3 for cubic as cubic is optimal
    List<double[]> points;
    double[] knots; //knot vector

    /**
     * @param points - list of control points
     * @param degree - degree of the spline
     * Note: Number of control points must be >= degree + 1!!!
     */
    public BSplinePath(List<double[]> points, int degree) {
        if (points.size() < degree + 1) {
            throw new IllegalArgumentException("Invalid number of control points");
        }
        this.degree = degree;
        this.points = points;
        this.knots = buildClampedKnotVector(points.size(), degree);

    }
    /**
     * Constructor for convenience which takes degree as 3 for cubic
     * @param points - list of control points
     */
    public BSplinePath(List<double[]> points) {
        this(points, 3);
    }

    /**
     * @return - returns the x, y coordinate of the curve on that position value
     */
    public double[] getPoint(double t) {
        t = clamp(t, 0.0, 1.0 );
        double tScaled = mapToKnotDomain(t);
        return deBoor(tScaled, 0);

    }
    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    private double mapToKnotDomain(double t) {
        int n = points.size() - 1;
        double low = knots[degree];
        double high = knots[n + 1];
        return low + t * (high - low);
    }
    private int findKnotSpan(double t) {
        int n = points.size() - 1;
        int p = degree;

        if (t >= knots[n + 1]) {
            int k = n;
            while (k > p && knots[k] == knots[n + 1]) k--;
            return k;
        }

        //binary search
        int l = degree, r = points.size();
        while (l < r - 1) {
           int mid = (l + r) / 2;
           if (knots[mid] <= t) {
               l = mid;
           } else {
               r = mid;
           }
        }
        return l;
    }
    private double[] buildClampedKnotVector(int numPoints, int deg) {
        int m = numPoints + deg;
        double[] kv = new double[m + 1];

        for (int i = 0; i <= deg; i++) kv[i] = 0.0;

        int interior = m - 2 * deg;
        for (int i = 1; i <= interior; i++) {
            kv[deg + i] = (double) i / (interior + 1);
        }

        for (int i = m - deg; i <= m; i++) kv[i] = 1.0;

        return kv;
    }
    public double[] getTangent(double t) {
        t = clamp(t, 0.0, 1.0);
        double tScaled = mapToKnotDomain(t);
        return deBoor(tScaled, 1);
    }

    public double[] getSecondDerivative(double t) {
        t = clamp(t, 0.0, 1.0);
        double tScaled = mapToKnotDomain(t);
        return deBoor(tScaled, 2);
    }

    public double getCurvature(double t) {
        double[] d1 = getTangent(t);
        double[] d2 = getSecondDerivative(t);
        double num   = d1[0] * d2[1] - d1[1] * d2[0];
        double denom = Math.pow(d1[0] * d1[0] + d1[1] * d1[1], 1.5);
        return (denom < 1e-9) ? 0.0 : num / denom;
    }

    public double getHeading(double t) {
        double[] tan = getTangent(t);
        return Math.atan2(tan[1], tan[0]);
    }

    public double approximateLength(int samples) {
        double length = 0.0;
        double[] prev = getPoint(0.0);
        for (int i = 1; i <= samples; i++) {
            double[] curr = getPoint((double) i / samples);
            double dx = curr[0] - prev[0];
            double dy = curr[1] - prev[1];
            length += Math.sqrt(dx * dx + dy * dy);
            prev = curr;
        }
        return length;
    }
    public double tAtArcLength(double targetLength, double totalLength, int samples) {
        if (targetLength <= 0) return 0.0;
        if (targetLength >= totalLength) return 1.0;

        double accumulated = 0.0;
        double[] prev = getPoint(0.0);

        for (int i = 1; i <= samples; i++) {
            double tCurr = (double) i / samples;
            double[] curr = getPoint(tCurr);
            double dx = curr[0] - prev[0];
            double dy = curr[1] - prev[1];
            accumulated += Math.sqrt(dx * dx + dy * dy);
            if (accumulated >= targetLength) return tCurr;
            prev = curr;
        }
        return 1.0;
    }

    public List<double[]> samplePoints(int count) {
        List<double[]> pts = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            pts.add(getPoint((double) i / (count - 1)));
        }
        return pts;
    }

    // -------------------------------------------------------------------------
    // De Boor's Algorithm
    // -------------------------------------------------------------------------
    private double[] deBoor(double t, int derivativeOrder) {
        int n = points.size() - 1;
        int k = findKnotSpan(t);

        int p = degree;
        double[][] d = new double[p + 1][2];

        for (int j = 0; j <= p; j++) {
            int idx = k - p + j;
            idx = Math.max(0, Math.min(idx, n));
            d[j][0] = points.get(idx)[0];
            d[j][1] = points.get(idx)[1];
        }
        for (int order = 0; order < derivativeOrder; order++) {
            int deg = p - order;
            for (int j = 0; j < deg; j++) {
                int knotIdx = k - deg + 1 + j;
                double denom = knots[knotIdx + deg] - knots[knotIdx];
                double scale = (denom < 1e-12) ? 0.0 : (double) deg / denom;
                d[j][0] = scale * (d[j + 1][0] - d[j][0]);
                d[j][1] = scale * (d[j + 1][1] - d[j][1]);
            }
        }
        for (int r = derivativeOrder; r < p; r++) {
            int deg = p - r;
            for (int j = deg - 1; j >= 0; j--) {
                int knotIdx = k - deg + 1 + j;
                double denom = knots[knotIdx + deg] - knots[knotIdx];
                if (denom < 1e-12) continue;
                double alpha = (t - knots[knotIdx]) / denom;
                d[j][0] = (1.0 - alpha) * d[j][0] + alpha * d[j + 1][0];
                d[j][1] = (1.0 - alpha) * d[j][1] + alpha * d[j + 1][1];
            }
        }
        return new double[]{d[0][0], d[0][1]};
    }

    //getters
    public int getDegree() {
        return degree;
    }
    public List<double[]> getControlPoints() {
        return points;
    }
    public double[] getKnots() {
        return knots.clone();
    }
}
