package controllers.pidf;

import com.qualcomm.robotcore.util.ElapsedTime;

import controllers.Controller;

/**
 * PIDF feedback controller.
 * @author Xander Haemel -31616
 * @author Atharv G - 13085 Bionic Dutch
 */
public class PIDFController extends Controller {
   private double kP, kI, kD, kF;
   private double integralSum = 0.0;
   private final double iLimit = 1.0;

    /**
     * default constructor
     * @param kP is the Proportional term
     * @param kI is the Integral term
     * @param kD is the Derivative term
     * @param kF is the Feedforward term
     */
    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

    }

    /**
     * default constructor
     * @param coefficients are the coefficients for the pidf
     */
    public PIDFController(PIDFCoefficients coefficients) {
        this(coefficients.getP(),
                coefficients.getI(),
                coefficients.getD(),
                coefficients.getF());
    }

    /**
     * sets the pidf coefficients for the controller
     * @param kP is the Proportional Term
     * @param kI is the Integral Term
     * @param kD is the Derivative Term
     * @param kF is the Feedforward Term
     */
    public void setPIDFCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * returns the current PIDF coefficients, p, i, d, and f
     * @return type PIDFCoefficients
     */
    public PIDFCoefficients getCoefficients() {
        return new PIDFCoefficients(kP, kI, kD, kF);
    }

    /**
     *
     * @param error Difference between goal and current position.
     * @param lastError Error from the previous loop.
     * @param deltaTime Time elapsed since last loop in seconds.
     * @return the power to set your motor to
     */
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

    /**
     * resets the integral term
     */
    public void resetIntegral() {
        this.integralSum = 0;
    }
}
