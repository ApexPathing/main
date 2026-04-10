package controllers.VectorControllers;

import util.Vector;

/**
 * This is basically just a PID controller that uses vectors instead of scalar values. This has two
 * benefits:
 * 1) Cleaner follower code
 * 2) Utilizes the VectorController template for boilerplate safety
 * <p>
 * Author: DrPixelCat24 (7842 alum)
 **/
public class PDLVectorController extends VectorController {
    private double kP, kD, kL;

    /**
     * @param kP Proportional term in the controller
     * @param kD Derivative term in the controller
     * @param kL Lower limit (minimum power) term. Prevents controller from failing due to friction.
     **/
    public PDLVectorController(double kP, double kD, double kL) {
        super();
        this.kP = kP;
        this.kD = kD;
        this.kL = kL;
    }

    public void setPDLCoefficients(double kP, double kD, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kL = kL;
    }

    @Override
    protected Vector computeOutput(Vector error, Vector lastError, double deltaTime) {
        // Calculate squared distance to check against tolerance
        double distSq = error.getX() * error.getX() + error.getY() * error.getY();

        // --- PROPORTIONAL & LOWER LIMIT (kL) TERM ---
        Vector pTerm = error.multiply(kP);
        Vector lTerm = error.normalize().multiply(kL);
        Vector baseResponse = pTerm.add(lTerm);

        if (!timeAnomalyDetected) {
            // --- DERIVATIVE TERM ---
            Vector deltaError = error.subtract(lastError);
            Vector dTerm = deltaError.multiply(kD / deltaTime);

            return baseResponse.add(dTerm);
        } else {
            // First loop or anomaly detected; return just the P and Feedforward terms
            return baseResponse;
        }
    }
}