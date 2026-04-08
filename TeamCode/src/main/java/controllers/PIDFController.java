package controllers;

/**
 * PIDF feedback controller.
 * @author Xander Haemel -31616
 */
public class PIDFController extends Controller {
   private double kP, kI, kD, kF;
   private double integralSum = 0.0;
   private final double iLimit = 1.0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setPIDFCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }


    @Override
    protected double computeOutput(double error, double lastError, double deltaTime) {
        // P term (Square Root)
        double proportional = kP * error;

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
