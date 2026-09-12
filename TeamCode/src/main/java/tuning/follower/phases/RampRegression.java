package tuning.follower.phases;

import java.util.ArrayList;
import java.util.List;

/** Least-squares moving-friction fit: applied power = kS + kV * measured velocity. */
final class RampRegression {
    static final class Sample {
        final double velocity, power;
        boolean excluded;
        Sample(double velocity, double power) { this.velocity = velocity; this.power = power; }
    }

    static final class Fit {
        final int count;
        final double kS, kV, rSquared, span;
        Fit(int count, double kS, double kV, double rSquared, double span) {
            this.count = count; this.kS = kS; this.kV = kV;
            this.rSquared = rSquared; this.span = span;
        }
        boolean valid(double minimumSpan) {
            return count >= 20 && span >= minimumSpan && Double.isFinite(kS)
                    && Double.isFinite(kV) && kS >= 0 && kS < .9 && kV > 0
                    && Double.isFinite(rSquared) && rSquared >= .9;
        }
    }

    final List<Sample> samples = new ArrayList<>();
    private final double minimumVelocity;
    RampRegression(double minimumVelocity) { this.minimumVelocity = minimumVelocity; }

    boolean add(double velocity, double power) {
        if (!Double.isFinite(velocity) || !Double.isFinite(power)
                || velocity < minimumVelocity || power <= 0 || power > .9) { return false; }
        samples.add(new Sample(velocity, power));
        return true;
    }

    Fit fit() {
        int n = 0;
        double x = 0, y = 0, min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            n++; x += s.velocity; y += s.power;
            min = Math.min(min, s.velocity); max = Math.max(max, s.velocity);
        }
        if (n < 2) { return new Fit(n, Double.NaN, Double.NaN, Double.NaN, 0); }
        x /= n; y /= n;
        double xx = 0, xy = 0, yy = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double dx = s.velocity - x, dy = s.power - y;
            xx += dx * dx; xy += dx * dy; yy += dy * dy;
        }
        double slope = xy / xx;
        return new Fit(n, y - slope * x, slope, xy * xy / (xx * yy), max - min);
    }

    /** Operator-requested exclusion only; never silently deletes an inconvenient measurement. */
    boolean excludeWorst() {
        Fit fit = fit();
        if (fit.count <= 20 || !Double.isFinite(fit.kV)) { return false; }
        Sample worst = null;
        double largest = -1;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double residual = Math.abs(s.power - fit.kS - fit.kV * s.velocity);
            if (residual > largest) { largest = residual; worst = s; }
        }
        if (worst == null) { return false; }
        worst.excluded = true;
        return true;
    }

    void restore() { for (Sample s : samples) { s.excluded = false; } }
}
