package controllers.VectorControllers;

import util.Vector;

/**
 * This is basically just a SquID controller that uses vectors instead of scalar values. This has two
 * benefits:
 * 1) Cleaner follower code
 * 2) See VectorController.java
 *
 * Author: DrPixelCat24 (7842 alum)
 */

//TODO: VERIFY THIS CLASS (I vibecoded the conversion from Controller -> VectorController);
public class SquidVectorController extends VectorController {
    public double kP, kI, kD;

    private Vector integralSum;
    private Vector lastPosition;
    private long lastTime;
    private boolean firstRun = true;

    private final double motorDeadzone = 0.05;
    private final double iLimit = 0.25;

    public SquidVectorController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralSum = new Vector();
        this.lastPosition = new Vector();
        this.lastTime = System.nanoTime();
    }

    @Override
    public Vector calculate(Vector currentPosition) {
        long currentTime = System.nanoTime();
        double dT = (currentTime - lastTime) / 1_000_000_000.0;
        lastTime = currentTime;

        if (firstRun || dT <= 0) {
            lastPosition = currentPosition.copy();
            firstRun = false;
            return new Vector();
        }

        Vector error = goal.subtract(currentPosition);
        double dist = error.getMagnitude();

        // --- P TERM (The SquID part) ---
        Vector pTerm = new Vector();
        if (dist > 1E-6) {
            pTerm = error.multiply(kP * Math.sqrt(dist)); // Note: magnitude is always positive so this should work
        }

        // --- I TERM ---
        integralSum = integralSum.add(error.multiply(dT));

        // Clamp added by Chat. I don't know if we want this or not.
        if (integralSum.getMagnitude() > iLimit) {
            // normalize() shrinks it to length 1, then we scale it to the exact iLimit
            integralSum = integralSum.normalize().multiply(iLimit);
        }
        Vector iTerm = integralSum.multiply(kI);

        // --- D TERM (Protected against Derivative Kick) ---
        Vector deltaPos = currentPosition.subtract(lastPosition);
        Vector dTerm = deltaPos.div(dT).multiply(-kD);
        lastPosition = currentPosition.copy();

        // --- COMBINE ---
        Vector power = pTerm.add(iTerm).add(dTerm);
        double powerMag = power.getMagnitude();

        // --- DEADZONE & CLAMPING ---
        if (powerMag < motorDeadzone) {
            return new Vector();
        } else if (powerMag > 1.0) {
            // normalize() inherently clamps a vector's magnitude to exactly 1.0
            // while preserving its exact direction!
            return power.normalize();
        }

        return power;
    }
}