package controllers;

/**
 * A general purpose controller specifically made for controlling a robot.
 * <ul>
 * <li><b>kP</b> - proportional gain (how aggressively the controller responds to error)</li>
 * <li><b>kD</b> - derivative gain (how much the controller accounts for error rate of change)</li>
 * <li><b>kF</b> - feedforward term (a constant power added to help overcome static forces)</li>
 * <li><b>kL</b> - minimum power (a constant power added in the error's direction to overcome static forces)</li>
 * </ul>
 *
 * @see followers.P2PFollower
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class PDFLController extends Controller {
    private PDFLCoefficients coeffs;

    // region Coefficients class
    /**
     * A simple class to hold the PDFLCoefficients for the PDFLController. This makes it easier to set
     * and manage the PDFLCoefficients.
     */
    public static class PDFLCoefficients {
        public double kP, kD, kF, kL;

        /**
         * Creates a PDFLCoefficients object with the given values. Check the
         * {@link PDFLController controller} documentation for what each coefficient does.
         */
        public PDFLCoefficients(double kP, double kD, double kF, double kL) {
            this.kP = kP; this.kD = kD; this.kF = kF; this.kL = kL;
        }

        /**
         * Creates a PDFLCoefficients object with the given P, D, and L values, with the F term set to
         * 0. Check the {@link PDFLController controller} documentation for what each coefficient
         * does.
         */
        public PDFLCoefficients(double kP, double kD, double kL) { this(kP, kD, 0.0, kL); }

        /**
         * Creates a PDFLCoefficients object with all coefficients set to 0.
         */
        public PDFLCoefficients() { this(0.0, 0.0, 0.0, 0.0); }
    }
    // endregion

    // region Constructors and getters/setters
    /**
     * Creates a PDFLController.
     * @param coefficients the {@link PDFLCoefficients} to use for the controller
     */
    public PDFLController(PDFLCoefficients coefficients) { this.setCoefficients(coefficients); }

    /**
     * @param PDFLCoefficients the {@link PDFLCoefficients} to use for the controller
     */
    public void setCoefficients(PDFLCoefficients PDFLCoefficients) { this.coeffs = PDFLCoefficients; }

    /**
     * @return the current {@link PDFLCoefficients} being used by the controller
     */
    public PDFLCoefficients getCoefficients() { return this.coeffs; }
    // endregion

    @Override
    protected double computeOutput(double error, double lastError, double deltaTime) {
        double proportional = this.coeffs.kP * error;
        double minimum = this.coeffs.kL * Math.signum(error);
        double derivative = this.coeffs.kD * (timeAnomalyDetected ? 0.0 : (error - lastError) / deltaTime);
        return proportional + derivative + this.coeffs.kF + minimum;
    }
}