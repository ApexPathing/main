package controllers;

import geometry.Angle;

/**
 * A general purpose PD controller with static friction compensation specifically made for
 * controlling robot movement in a one-dimensional axis.
 *
 * <ul>
 * <li><b>kP</b> - proportional gain (how aggressively the controller responds to error)</li>
 * <li><b>kD</b> - derivative gain (how much the controller accounts for error rate of change)</li>
 * <li><b>kS</b> - minimum power (a constant power added in the error's direction to overcome
 * static forces)</li>
 * </ul>
 *
 * <p>The controller applies the full static-friction term outside a small target deadband and
 * removes it inside that deadband. This preserves breakaway authority without causing target
 * jitter.
 *
 * <p>Special thanks to Wolfpack Machina (18438) for inspiration for this controller
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author DrPixelCat - 7842 alum
 */
public class PDSController {
    /** Translational error where static-friction compensation turns off, in inches. */
    public static final double LINEAR_STATIC_DEADBAND = 0.25;
    /** Angular error where static-friction compensation turns off, in radians. */
    public static final double ANGULAR_STATIC_DEADBAND = Math.toRadians(0.75);
    private double staticDeadband = LINEAR_STATIC_DEADBAND;

    private PDSCoefficients coeffs;

    private boolean angularController = false;

    protected double target = 0.0;
    protected double lastError = 0.0;
    protected boolean timeAnomaly = false;
    private boolean firstRun = true;
    private long lastTimestamp;

    /**
     * A simple class to hold the PDSCoefficients for the PDSController.
     */
    public static class PDSCoefficients {
        public double kP, kD, kS;

        /**
         * Creates a PDSCoefficients object with the given values. Check the
         * {@link PDSController controller} documentation for what each coefficient does.
         */
        public PDSCoefficients(double kP, double kD, double kS) {
            this.kP = kP;
            this.kD = kD;
            this.kS = kS;
        }

        /**
         * Creates a PDSCoefficients object with all coefficients set to 0.
         */
        public PDSCoefficients() { this(0.0, 0.0, 0.0); }

        public void setkP(double kP) { this.kP = kP; }
        public void setkD(double kD) { this.kD = kD; }
        public void setkS(double kS) { this.kS = kS; }
    }

    /** @param coefficients the {@link PDSCoefficients} to use for the controller */
    public PDSController(PDSCoefficients coefficients) {
        this.setCoefficients(coefficients);
        this.lastTimestamp = System.nanoTime();
    }

    /** @param PDSCoefficients the {@link PDSCoefficients} to use for the controller */
    public void setCoefficients(PDSCoefficients PDSCoefficients) { this.coeffs = PDSCoefficients; }

    /** @return the current {@link PDSCoefficients} being used by the controller */
    public PDSCoefficients getCoefficients() { return this.coeffs; }

    /**
     * Sets the controller to be an angular controller. This should be called if the controller is
     * being used to control an angular value
     */
    public void setAngularController() {
        this.angularController = true;
        this.staticDeadband = ANGULAR_STATIC_DEADBAND;
    }

    /**
     * Resets the controller state. Call this right before starting a new movement to prevent
     * derivative kick and reset the timer.
     */
    public void reset() {
        this.firstRun = true;
        this.lastTimestamp = System.nanoTime();
    }

    /**
     * Calculates the output directly from a pre-calculated error.
     *
     * @param error The calculated error (Target - Current)
     * @return The control output
     */
    public double calculate(double error) {
        return calculate(error, true);
    }

    /** Calculates output while optionally suppressing static-friction compensation. */
    public double calculate(double error, boolean applyStaticCompensation) {
        long currentNano = System.nanoTime();

        // Nanoseconds to seconds
        double deltaTime = (currentNano - lastTimestamp) / 1_000_000_000.0;

        // Detect if loop is too fast (div by zero risk) or too slow (integral/derivative spike)
        timeAnomaly = deltaTime < 1E-6 || deltaTime > 0.15;

        double actualError = angularController ? Angle.wrap(error) : error; // -pi to pi for angular

        if (firstRun) {
            lastError = actualError; // Prevents derivative kick from 0
            timeAnomaly = true;
            firstRun = false;
        }

        double p = this.coeffs.kP * actualError;
        double d = this.coeffs.kD * (timeAnomaly ? 0.0 : (actualError - lastError) / deltaTime);
        double s = applyStaticCompensation ? staticCompensation(actualError) : 0.0;

        lastTimestamp = currentNano;
        lastError = actualError;

        return p + d + s;
    }

    /**
     * Calculates the output when the error rate is already measured.
     *
     * <p>This avoids differentiating noisy samples and is preferable when velocity is available.
     * For a fixed position target, pass {@code -measuredVelocity} as {@code errorRate}.</p>
     */
    public double calculate(double error, double errorRate) {
        return calculate(error, errorRate, true);
    }

    /** Uses a measured error rate while optionally suppressing static compensation. */
    public double calculate(double error, double errorRate, boolean applyStaticCompensation) {
        double actualError = angularController ? Angle.wrap(error) : error;
        double p = coeffs.kP * actualError;
        double d = coeffs.kD * errorRate;
        double s = applyStaticCompensation ? staticCompensation(actualError) : 0.0;

        lastTimestamp = System.nanoTime();
        lastError = actualError;
        firstRun = false;
        timeAnomaly = false;
        return p + d + s;
    }

    /** Returns full static compensation outside the target deadband and zero inside it. */
    private double staticCompensation(double error) {
        return Math.abs(error) - staticDeadband > 1e-12
                ? Math.copySign(coeffs.kS, error)
                : 0.0;
    }
}
