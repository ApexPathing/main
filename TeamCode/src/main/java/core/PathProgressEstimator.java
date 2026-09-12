package core;

/** Filters the derivative of closest-point path progress while rejecting timing discontinuities. */
final class PathProgressEstimator {
    private static final double FILTER_SECONDS = 0.08;
    private static final double MAX_SAMPLE_SECONDS = 0.10;

    private double previousDistanceIn = Double.NaN;
    private long previousSampleNanos;
    private double velocityInPerSecond;

    double update(double distanceTraveled, double measuredTangentialVelocity, long nowNanos) {
        if (!Double.isFinite(previousDistanceIn) || previousSampleNanos == 0L) {
            previousDistanceIn = distanceTraveled;
            previousSampleNanos = nowNanos;
            velocityInPerSecond = measuredTangentialVelocity;
            return velocityInPerSecond;
        }

        double dt = (nowNanos - previousSampleNanos) * 1e-9;
        double distanceDelta = distanceTraveled - previousDistanceIn;
        previousDistanceIn = distanceTraveled;
        previousSampleNanos = nowNanos;
        if (dt <= 1e-4 || dt > MAX_SAMPLE_SECONDS || distanceDelta < 0.0) {
            return velocityInPerSecond;
        }

        double sample = distanceDelta / dt;
        double alpha = dt / (FILTER_SECONDS + dt);
        velocityInPerSecond += alpha * (sample - velocityInPerSecond);
        return velocityInPerSecond;
    }

    void reset() {
        previousDistanceIn = Double.NaN;
        previousSampleNanos = 0L;
        velocityInPerSecond = 0.0;
    }
}
