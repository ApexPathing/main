package controllers;

/**
 * A general purpose controller specifically made for controlling a robot.
 * <ul>
 * <li><b>kP</b> - proportional gain (how aggressively the controller responds to error)</li>
 * <li><b>kD</b> - derivative gain (how much the controller accounts for error rate of change)</li>
 * <li><b>kS</b> - minimum power (a constant power added in the error's direction to overcome static forces)</li>
 * </ul>
 *
 * @see followers.P2PFollower
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Joel - 7842 Browncoats Alumni
 */
public class PDSController extends Controller {
    private PDSCoefficients coeffs;

    // region Coefficients class
    /**
     * A simple class to hold the PDSCoefficients for the PDSController.
     */
    public static class PDSCoefficients {
        public double kP, kD, kS, kSDeadzone;

        /**
         * Creates a PDSCoefficients object with the given values. Check the
         * {@link PDSController controller} documentation for what each coefficient does.
         */
        public PDSCoefficients(double kP, double kD, double kS, double kSDeadzone) {
            this.kP = kP; this.kD = kD; this.kS = kS; this.kSDeadzone = kSDeadzone;
        }

        /**
         * Creates a PDFLCoefficients object with all coefficients set to 0.
         */
        public PDSCoefficients() { this(0.0, 0.0, 0.0, 0.0); }
    }
    // endregion

    // region Constructors and getters/setters
    /**
     * Creates a PDFLController.
     * @param coefficients the {@link PDSCoefficients} to use for the controller
     */
    public PDSController(PDSCoefficients coefficients) { this.setCoefficients(coefficients); }

    /**
     * @param PDSCoefficients the {@link PDSCoefficients} to use for the controller
     */
    public void setCoefficients(PDSCoefficients PDSCoefficients) { this.coeffs = PDSCoefficients; }

    /**
     * @return the current {@link PDSCoefficients} being used by the controller
     */
    public PDSCoefficients getCoefficients() { return this.coeffs; }
    // endregion

    @Override
    protected double computeOutput(double error, double lastError, double deltaTime) {
        double proportional = this.coeffs.kP * error;
        double derivative = this.coeffs.kD * (timeAnomalyDetected ? 0.0 : (error - lastError) / deltaTime);
        double minimum = this.coeffs.kS * Math.signum(error) * (Math.abs(error) > this.coeffs.kSDeadzone ? 1.0 : 0.0);
        return proportional + derivative + minimum;
    }
}