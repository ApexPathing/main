package controllers;

/**
 * An all-purpose controller for robotics. It removes I term and replaces it with a minimum power term.
 */
public class PDFLController extends Controller{
    private double kP, kD, kF, kL;
    public PDFLController(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void setPDFLCoefficients(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    @Override
    protected double computeOutput(double error, double lastError, double deltaTime) {
        double proportional = kP * error;
        double minimum = kL * Math.signum(error);
        double derivative = kD * (timeAnomalyDetected ? 0.0 : (error - lastError) / deltaTime);
        return proportional + derivative + kF + minimum;
    }
}
