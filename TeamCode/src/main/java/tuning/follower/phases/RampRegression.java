package tuning.follower.phases;

import java.util.ArrayList;
import java.util.List;

/** Moving-friction regression, optionally separating inertial power with paired ramps. */
final class RampRegression {
    static final class Sample {
        final double velocity, power, acceleration;
        boolean excluded;
        Sample(double velocity, double power, double acceleration) {
            this.velocity = velocity; this.power = power; this.acceleration = acceleration;
        }
    }

    static final class Fit {
        final int count;
        final double kS, kV, rSquared, span;
        double kA;
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
    private boolean modelInertia;
    RampRegression(double minimumVelocity) { this.minimumVelocity = minimumVelocity; }

    boolean add(double velocity, double power) {
        if (!Double.isFinite(velocity) || !Double.isFinite(power)
                || velocity < minimumVelocity || power <= 0 || power > .9) { return false; }
        samples.add(new Sample(velocity, power, 0));
        return true;
    }

    boolean add(double velocity, double power, double acceleration) {
        modelInertia = true;
        if (!Double.isFinite(velocity) || !Double.isFinite(power)
                || !Double.isFinite(acceleration) || velocity < minimumVelocity
                || power < 0 || power > .9) { return false; }
        samples.add(new Sample(velocity, power, acceleration));
        return true;
    }

    Fit fit() {
        if (modelInertia) { return fitInertia(); }
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

    /** Fit moving power = kS + kV*v + kA*a; both ramp directions identify the intercept. */
    private Fit fitInertia() {
        int n = 0, accelerating = 0, braking = 0;
        double v = 0, a = 0, p = 0, min = Double.POSITIVE_INFINITY, max = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            n++; v += s.velocity; a += s.acceleration; p += s.power;
            if (s.acceleration > 0) { accelerating++; }
            if (s.acceleration < 0) { braking++; }
            min = Math.min(min, s.velocity); max = Math.max(max, s.velocity);
        }
        Fit invalid = new Fit(n, Double.NaN, Double.NaN, Double.NaN, max - min);
        if (accelerating < 10 || braking < 10) { return invalid; }
        v /= n; a /= n; p /= n;
        double vv = 0, aa = 0, va = 0, vp = 0, ap = 0, pp = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double dv = s.velocity-v, da = s.acceleration-a, dp = s.power-p;
            vv += dv*dv; aa += da*da; va += dv*da; vp += dv*dp; ap += da*dp; pp += dp*dp;
        }
        double determinant = vv*aa-va*va;
        if (determinant <= 1e-6 * vv*aa || vv <= 0 || aa <= 0 || pp <= 0) { return invalid; }
        double kv = (vp*aa-ap*va)/determinant;
        double ka = (ap*vv-vp*va)/determinant;
        if (!Double.isFinite(ka) || ka < 0) { return invalid; }
        double ks = p-kv*v-ka*a;
        double residual = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double error = s.power-ks-kv*s.velocity-ka*s.acceleration;
            residual += error*error;
        }
        Fit result = new Fit(n, ks, kv, 1-residual/pp, max-min);
        result.kA = ka;
        return result;
    }

    /** Operator-requested exclusion only; never silently deletes an inconvenient measurement. */
    boolean excludeWorst() {
        Fit fit = fit();
        if (fit.count <= 20 || !Double.isFinite(fit.kV)) { return false; }
        Sample worst = null;
        double largest = -1;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double residual = Math.abs(s.power - fit.kS - fit.kV * s.velocity
                    - fit.kA * s.acceleration);
            if (residual > largest) { largest = residual; worst = s; }
        }
        if (worst == null) { return false; }
        worst.excluded = true;
        return true;
    }

    void restore() { for (Sample s : samples) { s.excluded = false; } }
}
