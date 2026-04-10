package controllers;

/**
 * A squID controller for follower
 * It takes the square root of the P, but otherwise it's pretty normal for a controller
 * @author Xander Haemel - 31616
 */
public class SquIDController extends Controller {
    public double kSq, kI, kD;

    private double integralSum = 0;
    private final double iLimit = 1.0; // TODO: confirm this value

    /**
     * Coefficients:
     * @param kSq Sqrt gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public SquIDController(double kSq, double kI, double kD) {
        super();
        this.kSq = kSq;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSquIDCoefficients(double kSq, double kI, double kD) {
        this.kSq = kSq;
        this.kI = kI;
        this.kD = kD;
    }

    @Override
    protected double computeOutput(double error, double lastError, double deltaTime) {
        // P term (Square Root)
        double proportional = kSq * Math.sqrt(Math.abs(error)) * Math.signum(error);

        if (!timeAnomalyDetected) {
            // I term
            integralSum += error * deltaTime;
            if (integralSum > iLimit) integralSum = iLimit;
            if (integralSum < -iLimit) integralSum = -iLimit;
            double integral = kI * integralSum;

            // D term
            double derivative = kD * ((error - lastError) / deltaTime);

            return proportional + integral + derivative;
        } else {
            return proportional;
        }
    }

    public void resetIntegral() {
        this.integralSum = 0;
    }
}