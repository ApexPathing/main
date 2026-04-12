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
   private double currentPos = 0.0;
   private double targetPos = 0.0;
   private double lastError = 0.0;

   private ElapsedTime timer;
   private double lastRecordedTime = 0.0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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

    public void setCurrentPos(double pos) {
        currentPos = pos;
    }
    public void setTargetPos(double pos) {
        targetPos = pos;
    }


    @Override
    protected double computeOutput(double error) {
        // P term (Square Root)
        double proportional = kP * error;
        double deltaTime = timer.time() - lastRecordedTime;

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

        lastError = error;
        lastRecordedTime = timer.time();
    }

    public double computeOutput() {
        return this.computeOutput(
                targetPos - currentPos
        );
    }

    public void resetIntegral() {
        this.integralSum = 0;
    }
}
