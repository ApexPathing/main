package controllers.pidf;

/**
 * @author Atharv G - 13085 Bionic Dutch
 * @author Xander Haemel - 31616 404 Not Found
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

    /**
     * gets the kP
     * @return the current kP value
     */
    public double getP() {
        return kP;
    }

    /**
     * gets the kI
     * @return the current kI value
     */
    public double getI() {
        return kI;
    }

    /**
     * gets the kD
     * @return the current kD value
     */
    public double getD() {
        return kD;
    }

    /**
     * gets the kF
     * @return the current kF value
     */
    public double getF() {
        return kF;
    }
}
