package controllers;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class Controller {
    protected double lastError = 0.0;
    protected double motorDeadzone = 0.05;
    protected boolean timeAnomalyDetected = false;
    private long lastTimestamp;
    private boolean hasRun = false;
    private boolean angularController = false;
    private boolean isAtTarget = false;
    private double tolerance;

    public Controller() {
        this.lastTimestamp = System.nanoTime();
    }

    public void setDeadzone(double deadzone) {
        this.motorDeadzone = deadzone;
    }

    public void useAsAngularController() {
        this.angularController = true;
    }

    public void useAsDefaultController() {
        this.angularController = false;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public boolean isAtTarget() {
        return isAtTarget;
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

    public double calculate(double error) {
        long currentNano = System.nanoTime();
        // Convert nanoseconds to seconds for standard unit gains
        double deltaTime = (currentNano - lastTimestamp) / 1_000_000_000.0;

        // Detect if loop is too fast (div by zero risk) or too slow (integral/derivative spike)
        timeAnomalyDetected = deltaTime < 1E-6 || deltaTime > 0.15;

        double actualError = angularController ?
                AngleUnit.normalizeRadians(error) :
                error;

        isAtTarget = Math.abs(actualError) < tolerance;

        // Initialize lastError on first run to prevent derivative kick from 0
        if (!hasRun) {
            deltaTime = 0.0;
            lastError = actualError;
            hasRun = true;
        }

        // Subclass-specific calculation (P, I, D, F, etc.)
        double rawPower = computeOutput(actualError, lastError, deltaTime);

        // Update state for next iteration
        lastTimestamp = currentNano;
        lastError = actualError;

        // Apply deadzone to prevent jitters and humming close to target
        if (Math.abs(rawPower) < motorDeadzone) {
            return 0;
        }

        // Constrain output to standard motor range [-1.0, 1.0]
        return Math.max(-1.0, Math.min(1.0, rawPower));
    }

    /**
     * @param error Difference between goal and current position.
     * @param lastError Error from the previous loop.
     * @param deltaTime Time elapsed since last loop in seconds.
     */
    protected abstract double computeOutput(double error, double lastError, double deltaTime);
}