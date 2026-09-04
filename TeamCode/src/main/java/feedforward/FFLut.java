package feedforward;

import androidx.annotation.NonNull;

/**
 * Linear lookup table for generated feedforward states.
 *
 * <p>
 * The generator samples a path into rows of {@link MotionParameters}. The follower later asks
 * for the row at its current path progression; this class linearly interpolates between the two
 * neighboring samples so the target does not jump from row to row.
 * </p>
 *
 * @author DrPixelCat - 7842 alum
 */
public class FFLut {
    /** Ordered samples of the path-relative state [v, a, omega, alpha]. */
    private final MotionParameters[] params;

    /**
     * Wraps generated profile samples.
     *
     * @param generatedParams ordered motion profile samples
     */
    public FFLut(MotionParameters[] generatedParams) {
        if (generatedParams == null || generatedParams.length == 0) {
            throw new IllegalArgumentException("A feedforward LUT requires at least one sample.");
        }
        for (int i = 1; i < generatedParams.length; i++) {
            if (generatedParams[i].getProgression() < generatedParams[i-1].getProgression()) {
                throw new IllegalArgumentException(
                        "Feedforward LUT progression keys must be ordered from low to high."
                );
            }
        }
        this.params = generatedParams;
    }

    /**
     * Returns an interpolated feedforward target at the requested progression.
     *
     * <p>Interpolation uses {@code y = y0 + fraction * (y1 - y0)}, where
     * {@code fraction = (progression - s0) / (s1 - s0)}. Each kinematic component is blended
     * independently. The fraction is derived from the requested displacement/progression; it is
     * not elapsed time or a path parameter.
     *
     * @param progression path progression/distance key to query
     * @return interpolated motion parameters for the follower
     */
    public MotionParameters getFFParams(double progression) {
        if (params.length == 1 || progression <= params[0].getProgression()) {
            return copyOf(params[0]);
        }

        MotionParameters last = params[params.length - 1 ];
        if (progression >= last.getProgression()) { return copyOf(last); }

        // Find the first sample at or beyond the requested progression.
        for (int i = 1; i < params.length; i++) {
            if (progression <= params[i].getProgression()) {
                MotionParameters params1 = params[i - 1];
                MotionParameters params2 = params[i];
                double s0 = params1.getProgression();
                double s1 = params2.getProgression();
                double denominator = s1 - s0;

                if (Math.abs(denominator) < 1e-9) { return copyOf(params2); }

                double interpolationFraction = (progression - s0) / denominator;
                return getFFParams(params1, interpolationFraction, params2, progression);
            }
        }
        return copyOf(last);
    }

    /** Returns a trajectory state interpolated by elapsed time. */
    public MotionParameters getFFParamsByTime(double timeSeconds) {
        if (params.length == 1 || timeSeconds <= params[0].getTimeSeconds()) {
            return copyOf(params[0]);
        }
        MotionParameters last = params[params.length - 1];
        if (timeSeconds >= last.getTimeSeconds()) { return copyOf(last); }

        for (int i = 1; i < params.length; i++) {
            if (timeSeconds <= params[i].getTimeSeconds()) {
                MotionParameters before = params[i - 1];
                MotionParameters after = params[i];
                double dt = after.getTimeSeconds() - before.getTimeSeconds();
                if (dt <= 1e-9) { return copyOf(after); }
                double fraction = (timeSeconds - before.getTimeSeconds()) / dt;
                return interpolate(before, fraction, after);
            }
        }
        return copyOf(last);
    }

    /** @return final elapsed-time key, or zero for a displacement-only profile */
    public double getDurationSeconds() { return params[params.length - 1].getTimeSeconds(); }

    /**
     * Blends two neighboring rows of the lookup table.
     *
     * @param params1 lower/previous sample
     * @param interpolationFraction interpolation factor between 0 and 1 in normal use
     * @param params2 upper/next sample
     * @return linearly interpolated parameters
     */
    @NonNull
    private static MotionParameters getFFParams(MotionParameters params1,
                                                double interpolationFraction,
                                                MotionParameters params2, double progression) {
        MotionParameters result = interpolate(params1, interpolationFraction, params2);
        result.setDistAlongCurve(progression);
        return result;
    }

    private static MotionParameters interpolate(MotionParameters params1,
                                                double interpolationFraction,
                                                MotionParameters params2) {
        double interpTransVel = params1.getTangentialVel() + interpolationFraction *
                        (params2.getTangentialVel() - params1.getTangentialVel());
        double interpTransAccel = params1.getTangentialAccel() + interpolationFraction *
                        (params2.getTangentialAccel() - params1.getTangentialAccel());
        double interpAngVel = params1.getAngularVel() + interpolationFraction *
                        (params2.getAngularVel() - params1.getAngularVel());
        double interpAngAccel = params1.getAngularAccel() + interpolationFraction *
                        (params2.getAngularAccel() - params1.getAngularAccel());

        double interpProgression = params1.getProgression() + interpolationFraction *
                (params2.getProgression() - params1.getProgression());
        double interpTime = params1.getTimeSeconds() + interpolationFraction *
                (params2.getTimeSeconds() - params1.getTimeSeconds());
        double interpMotorPower = params1.getMotorPower() + interpolationFraction *
                (params2.getMotorPower() - params1.getMotorPower());
        MotionParameters result = new MotionParameters(interpTransVel, interpTransAccel,
                interpAngVel, interpAngAccel, interpProgression).setTimeSeconds(interpTime);
        result.setMotorPower(interpMotorPower);
        return result;
    }

    private static MotionParameters copyOf(MotionParameters params) {
        MotionParameters result = new MotionParameters(
                params.getTangentialVel(), params.getTangentialAccel(), params.getAngularVel(),
                params.getAngularAccel(), params.getProgression()
        ).setTimeSeconds(params.getTimeSeconds());
        result.setMotorPower(params.getMotorPower());
        return result;
    }
}
