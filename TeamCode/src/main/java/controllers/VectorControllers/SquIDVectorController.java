package controllers.VectorControllers;

import util.Vector;

/**
 * This is basically just a SquID controller that uses vectors instead of scalar values. This has two
 * benefits:
 * 1) Cleaner follower code
 * 2) See VectorController.java
 * <p>
 * Author: DrPixelCat24 (7842 alum)
 **/
public class SquIDVectorController extends VectorController {
    private final double MOTOR_DEADZONE = 0.05 * 0.05; // Squared for better efficiency
    public double kSq, kD;
    private long lastTime;
    private boolean firstRun = true;
    private Vector lastErr;

    //TODO: Test with vs. without integral component. Hypothesis is that it shouldn't be needed.
    private double kI;
    private Vector integralSum;
    private Vector integral(double dT, Vector dE) {
        integralSum = integralSum.add(dE.multiply(dT));
        return integralSum.multiply(kI);
    }

    /**
     * NOTE: NO I COMPONENT NEEDED
     * @param kSq Sqrt gain
     * @param kD Derivative gain
     */
    public SquIDVectorController(double kSq, double kD) {
        this.kSq = kSq;
        this.kD = kD;
        this.lastTime = System.nanoTime();
    }

    public void setSquIDCoefficients(double kSq, double kD) {
        this.kSq = kSq;
        this.kD = kD;
    }

    // TODO: Remove below constructor AND following function if hypothesis is confirmed
    public SquIDVectorController(double kSq, double kI, double kD) {
        this(kSq, kD);
        this.kI = kI;
    }

    public void setSquIDCoefficients(double kSq, double kD, double kI) {
        this.kSq = kSq;
        this.kD = kD;
        this.kI = kI;
    }

    @Override
    public Vector calculate(Vector currentPosition) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;
        lastTime = currentTime;

        Vector error = goal.subtract(currentPosition);

        if (firstRun) {
            lastErr = error;
            firstRun = false;
            deltaTime = 0.0;
        }

        double distanceToGoal = error.getMagnitude();

        // --------------- Sqrt response ---------------
        double squResponse = kSq * Math.sqrt(distanceToGoal); // Mag is always (+) so this is fine

        // --------------- Derivative response ---------------
        Vector deltaError = error.subtract(lastErr);
        Vector dResponse = new Vector();

        if (deltaTime > 1E-6) {
            dResponse = deltaError.multiply(kD / deltaTime);
        }

        lastErr = error;

        // --------------- Integral Response --------------- TODO: Remove or clean based on testing conclusions
        dResponse.add(integral(deltaTime, deltaError));

        Vector response = error.normalize().multiply(squResponse).add(dResponse);

        if (response.getMagnitudeSquared() < MOTOR_DEADZONE) {
            return new Vector();
        } else {
            return response;
        }
    }
}