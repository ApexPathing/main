package controllers.vector;

import util.Vector;

/**
 * This is basically just a SquID controller that uses vectors instead of scalar values. This has two
 * benefits:
 * 1) Cleaner follower code
 * 2) Utilizes the VectorController template for boilerplate safety
 * <p>
 * Author: DrPixelCat24 (7842 alum)
 **/

// TODO: Test this class with and without integral to assess possibility of removal.
public class SquIDVectorController extends VectorController {
    public double kSq, kI, kD;
    private Vector integralSum = new Vector();

    /**
     * Constructor for PD control (Assuming hypothesis: NO I COMPONENT NEEDED)
     * @param kSq Sqrt gain
     * @param kD Derivative gain
     */
    public SquIDVectorController(double kSq, double kD) {
        super();
        this.kSq = kSq;
        this.kD = kD;
        this.kI = 0.0; // Explicitly zeroed out
    }

    /**
     * Constructor for full SquID control (for testing the hypothesis)
     */
    public SquIDVectorController(double kSq, double kI, double kD) {
        super();
        this.kSq = kSq;
        this.kI = kI;
        this.kD = kD;
    }

    public void setCoefficients(double kSq, double kD) {
        this.kSq = kSq;
        this.kD = kD;
    }

    public void setCoefficients(double kSq, double kI, double kD) {
        this.kSq = kSq;
        this.kI = kI;
        this.kD = kD;
    }

    @Override
    protected Vector computeOutput(Vector error, Vector lastError, double deltaTime) {
        double distanceToGoal = error.getMagnitude();

        // --------------- Sqrt response ---------------
        double squResponse = kSq * Math.sqrt(distanceToGoal);
        Vector proportional = error.normalize().multiply(squResponse);

        if (!timeAnomalyDetected) {
            // --------------- Derivative response ---------------
            Vector deltaError = error.subtract(lastError);
            Vector derivative = deltaError.multiply(kD / deltaTime);

            // --------------- Integral Response ---------------
            Vector integral = new Vector();
            if (kI != 0.0) {
                integralSum = integralSum.add(error.multiply(deltaTime));
                integral = integralSum.multiply(kI);
            }

            return proportional.add(derivative).add(integral);
        } else {
            return proportional;
        }
    }

    /**
     * Resets the accumulated integral sum.
     * Useful when starting a new trajectory to prevent integral windup.
     */
    public void resetIntegral() {
        this.integralSum = new Vector();
    }
}