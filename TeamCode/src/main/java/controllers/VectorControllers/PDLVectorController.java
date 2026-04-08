package controllers.VectorControllers;

import util.Vector;

/**
 * This is basically just a PID controller that uses vectors instead of scalar values. This has two
 * benefits:
 * 1) Cleaner follower code
 * 2) See VectorController.java
 * <p>
 * Author: DrPixelCat24 (7842 alum)
 **/
public class PDLVectorController extends VectorController {
    private double kP, kD, kL, kL_tolSq;
    Vector goal;
    private long lastTime;
    private Vector lastErr;
    private boolean firstRun = true;

    /**
    * @param kP Proportional term in the controller
     * @param kD Derivative term in the controller
     * @param kL Lower limit (minimum power) term. Prevents controller from failing due to friction.
     * @param kLTol Tolerance for kL - if backlash is present it can cause jitters without this term.
     **/
    public PDLVectorController(double kP, double kD, double kL, double kLTol) {
        this.kP = kP;
        this.kD = kD;
        this.kL = kL;
        this.kL_tolSq = kLTol * kLTol;

        this.lastErr = new Vector();
        this.goal = new Vector();
    }

    public void setPDLCoefficients(double kP, double kD, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kL = kL;
    }

    public static Vector scaleToDirection(Vector target, double distSq, double desiredMagnitude) {
        if (distSq < 1e-8) {
            return new Vector(0, 0);
        }
        double scale = desiredMagnitude / Math.sqrt(distSq);
        return new Vector(target.getX() * scale, target.getY() * scale);
    }

    @Override
    public Vector calculate(Vector currentPosition) {
        long currentTime = System.nanoTime();

        double dT = (currentTime - lastTime) / 1_000_000_000.0;
        lastTime = currentTime;
        Vector error = goal.subtract(currentPosition);

        if (firstRun) {
            lastErr = error;
            firstRun = false;
            dT = 0.0;
        }

        double distSq = error.getX() * error.getX() + error.getY() * error.getY();

        // --- PROPORTIONAL & LIMIT TERM ---
        Vector pTerm;
        if (distSq > kL_tolSq) {
            pTerm = error.multiply(kP).add(scaleToDirection(error, distSq, kL));
        } else {
            lastErr = error;
            return new Vector();
        }

        // --- DERIVATIVE TERM ---
        Vector dE = error.subtract(lastErr);
        Vector dTerm = new Vector();

        if (dT > 1E-6) {
            dTerm = dE.multiply(kD / dT);
        }

        lastErr = error;

        return pTerm.add(dTerm);
    }
}