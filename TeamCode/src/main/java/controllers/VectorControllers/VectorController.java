package controllers.VectorControllers;

import util.Vector;

/**
 * Vector controllers allow more accuracy for long trajectories. Since some feedback controllers
 * (i.e. SquID) have non-linear behavior, this can result in undesirable behavior in the form of
 * curved trajectories. This fixes that issue by pointing all the feedback power straight in the
 * direction of the error.
 * <p>
 * Author: DrPixelCat24 (7842 alum)
 */
public abstract class VectorController {
    protected Vector lastError = new Vector();
    protected double motorDeadzone = 0.05;
    protected boolean timeAnomalyDetected = false;
    private long lastTimestamp = 0;
    private boolean hasRun = false;
    private boolean isAtTarget = false;
    private double tolerance;

    public VectorController() {
        this.lastTimestamp = System.nanoTime();
    }

    public void setDeadzone(double deadzone) {
        this.motorDeadzone = deadzone;
    }
    public boolean isAtTarget() {
        return isAtTarget;
    }
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance * tolerance;
    }

    /**
     * Resets the controller state. Call this right before starting a new movement
     * to prevent derivative kick and reset the timer.
     */
    public void reset() {
        this.isAtTarget = false;
        this.hasRun = false;
        this.lastTimestamp = System.nanoTime();
    }

    /**
     * The template method that handles standard vector controller boilerplate.
     */
    public Vector calculate(Vector error) {
        isAtTarget = error.getMagnitudeSquared() < tolerance;

        long currentNano = System.nanoTime();
        double deltaTime = (currentNano - lastTimestamp) / 1_000_000_000.0;

        timeAnomalyDetected = deltaTime < 1E-6 || deltaTime > 0.15;

        // Prevent division by zero or explosive D-terms
        if (deltaTime < 1E-6) {
            return new Vector();
        }

        if (!hasRun) {
            lastError = error;
            deltaTime = 0.0;
            hasRun = true;
        }

        Vector rawPower = computeOutput(error, lastError, deltaTime);

        lastTimestamp = currentNano;
        lastError = error;

        double powerMag = rawPower.getMagnitude();

        if (powerMag < motorDeadzone) {
            return new Vector();
        }

        if (powerMag > 1.0) {
            return rawPower.normalize();
        }

        return rawPower;
    }

    /**
     * Subclasses must implement this to define their specific vector math.
     */
    protected abstract Vector computeOutput(Vector error, Vector lastError, double deltaTime);
}