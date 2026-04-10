package controllers;

public abstract class Controller {
    protected double goal = 0.0;
    protected double lastError = 0.0;
    protected double motorDeadzone = 0.05;
    protected boolean timeAnomalyDetected = false;
    private long lastTimestamp;
    private boolean hasRun = false;

    public Controller() {
        this.lastTimestamp = System.nanoTime();
    }

    public void setGoal(double newGoal) {
        this.goal = newGoal;
    }

    public void setDeadzone(double deadzone) {
        this.motorDeadzone = deadzone;
    }

    /**
     * Resets the controller state. Call this right before starting a new movement
     * to prevent derivative kick and reset the timer.
     */
    public void reset() {
        this.hasRun = false;
        this.lastTimestamp = System.nanoTime();
    }

    public synchronized double calculate(double currentPosition) {
        long currentNano = System.nanoTime();
        double deltaTime = (currentNano - lastTimestamp) / 1_000_000_000.0;

        timeAnomalyDetected = deltaTime < 1E-6 || deltaTime > 0.15;

        double error = goal - currentPosition;

        if (!hasRun) {
            deltaTime = 0.0;
            lastError = error;
            hasRun = true;
        }

        double rawPower = computeOutput(error, lastError, deltaTime);

        lastTimestamp = currentNano;
        lastError = error;

        if (Math.abs(rawPower) < motorDeadzone) {
            return 0;
        }

        return Math.max(-1.0, Math.min(1.0, rawPower));
    }

    protected abstract double computeOutput(double error, double lastError, double deltaTime);
}