package controllers.pidf;

/**
 * @author Atharv G - 13085 Bionic Dutch
 * Organizes PIDF controller constants
 */
public class PIDFCoefficients {
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    public PIDFCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }
}
