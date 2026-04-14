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

   private ElapsedTime timer;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

    }

    public PIDFController(PIDFCoefficients coefficients) {
        this(coefficients.getP(),
                coefficients.getI(),
                coefficients.getD(),
                coefficients.getF());
    }

    public void setPIDFCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public PIDFCoefficients getCoefficients() {
        return new PIDFCoefficients(kP, kI, kD, kF);
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
