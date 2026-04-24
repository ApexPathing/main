package controllers;

/**
 * A general purpose controller specifically made for controlling a robot.
 *
 * @see followers.P2PFollower
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class PDLController extends Controller {
    public static class Coefficients {
        public double kP, kD, kL;

        public Coefficients(double kP, double kD, double kL) {
            this.kP = kP; this.kD = kD; this.kL = kL;
        }

        public Coefficients() { this(0.0, 0.0, 0.0); }
    }

    private Coefficients coeffs;

    public PDLController(Coefficients coefficients) { this.coeffs = coefficients; }

    public PDLController(double kP, double kD, double kL) { this(new Coefficients(kP, kD, kL)); }

    public PDLController() { this(0.0, 0.0, 0.0); }

    public void setPDLCoefficients(Coefficients coefficients) { this.coeffs = coefficients; }

    public void setPDLCoefficients(double kP, double kD, double kL) {
        this.coeffs.kP = kP; this.coeffs.kD = kD; this.coeffs.kL = kL;
    }

    @Override
    protected double computeOutput(double error, double lastError, double deltaTime) {
        double proportional = this.coeffs.kP * error;
        double minimum = this.coeffs.kL * Math.signum(error);
        double derivative = this.coeffs.kD * (timeAnomalyDetected ? 0.0 : (error - lastError) / deltaTime);
        return proportional + derivative + minimum;
    }
}