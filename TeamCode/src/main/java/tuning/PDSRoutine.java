package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import geometry.Angle;

/** Non-blocking coordinate-descent tuner shared by linear and angular PDS controllers. */
public final class PDSRoutine {
    // region Configuration

    /** Unit, safety, and gain bounds for one axis. */
    public static final class Config {
        final String name;
        final double minP, maxP, minD, maxD, step, tolerance, velocityTolerance, safetyLimit;
        final boolean angular;

        /** Creates an inches-based configuration. */
        public static Config linear(String name, double minP, double maxP, double minD,
                                    double maxD, double step, double tolerance,
                                    double velocityTolerance, double safetyLimit) {
            return new Config(name, minP, maxP, minD, maxD, step, tolerance,
                    velocityTolerance, safetyLimit, false);
        }

        /** Creates a radians-based configuration. */
        public static Config angular(String name, double minP, double maxP, double minD,
                                     double maxD, double step, double tolerance,
                                     double velocityTolerance, double safetyLimit) {
            return new Config(name, minP, maxP, minD, maxD, step, tolerance,
                    velocityTolerance, safetyLimit, true);
        }

        private Config(String name, double minP, double maxP, double minD, double maxD,
                       double step, double tolerance, double velocityTolerance,
                       double safetyLimit, boolean angular) {
            if (name == null || name.isEmpty() || !range(minP, maxP) || !range(minD, maxD) ||
                    !positive(step) || !positive(tolerance) || !positive(velocityTolerance) ||
                    !positive(safetyLimit) || safetyLimit <= step) {
                throw new IllegalArgumentException("Invalid PDS tuning configuration");
            }
            this.name = name;
            this.minP = minP;
            this.maxP = maxP;
            this.minD = minD;
            this.maxD = maxD;
            this.step = step;
            this.tolerance = tolerance;
            this.velocityTolerance = velocityTolerance;
            this.safetyLimit = safetyLimit;
            this.angular = angular;
        }

        private static boolean range(double min, double max) {
            return Double.isFinite(min) && min >= 0.0 && Double.isFinite(max) && max > min;
        }
    }

    /** Public optimizer state. */
    public enum PDSState { SETTLING, TUNING, TEST_READY, TEST_RUNNING, COMPLETE }
    private enum Evaluation { BASELINE, PLUS, MINUS }

    private static final double COMMAND_LIMIT = 0.85;
    private static final double TRIAL_TIMEOUT = 4.0;
    private static final double SETTLED_TIME = 0.25;
    private static final double BETWEEN_TRIALS = 0.25;
    private static final double MAX_VALID_DT = 0.08;
    private static final int TRIALS_PER_CANDIDATE = 2;
    private static final int MAX_COORDINATE_STEPS = 8;

    private final Config config;
    private final PDSController controller;
    private final ElapsedTime clock = new ElapsedTime();
    private final double[] gains = new double[2];
    private final double[] deltas = new double[2];
    private PDSState state = PDSState.SETTLING;
    private Evaluation evaluation = Evaluation.BASELINE;
    private boolean started;
    private double origin = Double.NaN, position, target, error, direction, nextDirection = 1.0;
    private double validTime, settledTime, errorIntegral, overshoot, saturatedTime, lastTime;
    private double candidateCost, bestCost = Double.POSITIVE_INFINITY;
    private int candidateTrials, coordinate, coordinateSteps, totalTrials;
    private String status = "Not started";

    /** Creates a Twiddle tuner from generic axis-specific PD guesses. */
    public PDSRoutine(Config config, double initialP, double initialD, double staticPower) {
        this.config = config;
        if (!Double.isFinite(initialP) || initialP < 0.0 ||
                !Double.isFinite(initialD) || initialD < 0.0 ||
                !Double.isFinite(staticPower) ||
                staticPower < 0.0 || staticPower >= COMMAND_LIMIT) {
            throw new IllegalArgumentException("Invalid PDS tuning seed");
        }
        controller = new PDSController(new PDSCoefficients(0.0, 0.0, staticPower));
        if (config.angular) { controller.setAngularController(); }
        gains[0] = Range.clip(initialP, config.minP, config.maxP);
        gains[1] = Range.clip(initialD, config.minD, config.maxD);
        deltas[0] = Math.max(gains[0] * 0.35, (config.maxP - config.minP) * 0.025);
        deltas[1] = Math.max(gains[1] * 0.35, (config.maxD - config.minD) * 0.025);
    }

    /** Starts with a paired baseline response. */
    public void start() {
        applyGains();
        started = true;
        origin = Double.NaN;
        evaluation = Evaluation.BASELINE;
        candidateTrials = coordinateSteps = totalTrials = coordinate = 0;
        candidateCost = 0.0;
        bestCost = Double.POSITIVE_INFINITY;
        nextDirection = 1.0;
        state = PDSState.SETTLING;
        status = "Measuring baseline";
        clock.reset();
    }

    /** Returns true after the user accepts the result. */
    public boolean isComplete() { return state == PDSState.COMPLETE; }

    /** Returns the current state. */
    public PDSState getState() { return state; }

    /** Returns a copy of the active coefficients. */
    public PDSCoefficients getCoefficients() {
        PDSCoefficients c = controller.getCoefficients();
        return new PDSCoefficients(c.kP, c.kD, c.kS);
    }

    /** Returns whether manual verification is available. */
    public boolean isTestReady() { return state == PDSState.TEST_READY; }

    /** Queues an alternating verification step. */
    public void requestTest() {
        if (state != PDSState.TEST_READY) { return; }
        candidateTrials = -1;
        state = PDSState.SETTLING;
        status = "Preparing user test";
        clock.reset();
    }

    /** Accepts the optimized result. */
    public void accept() {
        if (state == PDSState.TEST_READY) { state = PDSState.COMPLETE; }
    }

    // endregion
    // region Real-time state machine

    /** Advances one non-blocking control-loop tick. */
    public double update(double absolutePosition, double velocity) {
        if (!started) { throw new IllegalStateException("Call start() before update()"); }
        if (!Double.isFinite(absolutePosition) || !Double.isFinite(velocity)) {
            throw new IllegalStateException("PDS tuner received invalid localization");
        }
        if (Double.isNaN(origin)) { origin = absolutePosition; }
        position = relativePosition(absolutePosition, origin, config.angular);
        if (Math.abs(position) > config.safetyLimit) {
            throw new IllegalStateException(
                    "PDS tuner exceeded its safe travel limit: position=" + position +
                            ", limit=" + config.safetyLimit + ", target=" + target +
                            ", velocity=" + velocity + ", state=" + state
            );
        }
        if (state == PDSState.SETTLING) { return settle(velocity); }
        if (state == PDSState.TUNING || state == PDSState.TEST_RUNNING) {
            return measure(velocity);
        }
        return 0.0;
    }

    /** Waits for rest without blocking. */
    private double settle(double velocity) {
        if (clock.seconds() < BETWEEN_TRIALS || Math.abs(velocity) > config.velocityTolerance) {
            return 0.0;
        }
        direction = nextDirection;
        nextDirection = -nextDirection;
        // Always exercise the same two physical endpoints. Building the next target from the
        // measured stopping position accumulates residual error over dozens of trials and can
        // walk an otherwise stable out-and-back test beyond its original safety envelope.
        target = anchoredTarget(direction, config.step);
        validTime = settledTime = errorIntegral = overshoot = saturatedTime = lastTime = 0.0;
        controller.reset();
        state = candidateTrials < 0 ? PDSState.TEST_RUNNING : PDSState.TUNING;
        status = state == PDSState.TEST_RUNNING ? "Running user test" : "Evaluating gains";
        clock.reset();
        return 0.0;
    }

    /** Measures one response while rejecting lag-spike time intervals. */
    private double measure(double velocity) {
        double now = clock.seconds();
        double rawDt = now - lastTime;
        lastTime = now;
        if (rawDt > 0.15) {
            // Physics and localization may have advanced discontinuously. Retry this direction;
            // ignoring only dt would still leave its jumped position embedded in the response.
            nextDirection = direction;
            state = PDSState.SETTLING;
            status = "Lag spike discarded; retrying response";
            clock.reset();
            controller.reset();
            return 0.0;
        }
        double dt = rawDt > 0.0 && rawDt <= MAX_VALID_DT ? rawDt : 0.0;
        error = positionError(target, position, config.angular);
        if (dt > 0.0) {
            validTime += dt;
            double normalized = error / config.step;
            errorIntegral += normalized * normalized * dt;
            overshoot = Math.max(overshoot,
                    Math.max(0.0, direction * positionError(position, target, config.angular)));
        }
        boolean inside = Math.abs(error) <= config.tolerance &&
                Math.abs(velocity) <= config.velocityTolerance;
        settledTime = inside ? settledTime + dt : 0.0;
        double raw = controller.calculate(error, -velocity);
        double command = Range.clip(raw, -COMMAND_LIMIT, COMMAND_LIMIT);
        if (dt > 0.0 && Math.abs(raw) >= COMMAND_LIMIT) { saturatedTime += dt; }
        if (settledTime >= SETTLED_TIME || validTime >= TRIAL_TIMEOUT) {
            finishResponse(settledTime >= SETTLED_TIME);
            return 0.0;
        }
        return command;
    }

    /** Scores a tuning response or finishes a user test. */
    private void finishResponse(boolean settled) {
        double rms = Math.sqrt(errorIntegral / Math.max(validTime, 1e-9));
        double settling = settled ? Math.max(0.0, validTime - settledTime) : TRIAL_TIMEOUT;
        double overshootFraction = overshoot / config.step;
        double finalFraction = Math.abs(error) / config.step;
        double saturation = saturatedTime / Math.max(validTime, 1e-9);
        double cost = 5.0 * settling + 5.0 * rms +
                14.0 * overshootFraction * overshootFraction +
                10.0 * finalFraction * finalFraction + 0.5 * saturation +
                (settled ? 0.0 : 30.0);
        totalTrials++;
        if (state == PDSState.TEST_RUNNING) {
            candidateTrials = 0;
            state = PDSState.TEST_READY;
            status = "Test complete; X repeats, A accepts";
            return;
        }
        candidateCost += cost;
        if (++candidateTrials < TRIALS_PER_CANDIDATE) {
            state = PDSState.SETTLING;
            clock.reset();
            return;
        }
        evaluate(candidateCost / TRIALS_PER_CANDIDATE);
    }

    // endregion
    // region Twiddle

    /** Executes standard +delta/-delta coordinate-descent branching. */
    private void evaluate(double cost) {
        if (evaluation == Evaluation.BASELINE) {
            bestCost = cost;
            evaluation = Evaluation.PLUS;
            gains[coordinate] = clip(coordinate, gains[coordinate] + deltas[coordinate]);
        } else if (evaluation == Evaluation.PLUS && cost < bestCost) {
            bestCost = cost;
            deltas[coordinate] *= 1.15;
            nextCoordinate();
        } else if (evaluation == Evaluation.PLUS) {
            gains[coordinate] = clip(coordinate, gains[coordinate] - 2.0 * deltas[coordinate]);
            evaluation = Evaluation.MINUS;
        } else if (cost < bestCost) {
            bestCost = cost;
            deltas[coordinate] *= 1.15;
            nextCoordinate();
        } else {
            gains[coordinate] = clip(coordinate, gains[coordinate] + deltas[coordinate]);
            deltas[coordinate] *= 0.55;
            nextCoordinate();
        }
        applyGains();
        candidateCost = 0.0;
        candidateTrials = 0;
        if (coordinateSteps >= MAX_COORDINATE_STEPS || normalizedDelta() < 0.025) {
            state = PDSState.TEST_READY;
            status = "Twiddle complete; X tests, A accepts";
        } else {
            state = PDSState.SETTLING;
            clock.reset();
        }
    }

    /** Advances to the other gain coordinate. */
    private void nextCoordinate() {
        coordinate = (coordinate + 1) % 2;
        coordinateSteps++;
        evaluation = Evaluation.PLUS;
        // Never leave an unevaluated +delta applied when this completed coordinate ends tuning.
        if (coordinateSteps < MAX_COORDINATE_STEPS && normalizedDelta() >= 0.025) {
            gains[coordinate] = clip(coordinate, gains[coordinate] + deltas[coordinate]);
        }
    }

    /** Applies the current candidate. */
    private void applyGains() {
        controller.setCoefficients(new PDSCoefficients(
                gains[0], gains[1], controller.getCoefficients().kS));
    }

    private double clip(int index, double value) {
        return index == 0 ? Range.clip(value, config.minP, config.maxP)
                : Range.clip(value, config.minD, config.maxD);
    }

    private double normalizedDelta() {
        return deltas[0] / (config.maxP - config.minP) +
                deltas[1] / (config.maxD - config.minD);
    }

    // endregion
    // region Utilities and telemetry

    /** Returns relative position, wrapping heading in radians. */
    static double relativePosition(double absolute, double origin, boolean angular) {
        return angular ? Angle.wrap(absolute - origin) : absolute - origin;
    }

    /** Returns one of the two fixed out-and-back endpoints in origin-relative coordinates. */
    static double anchoredTarget(double direction, double step) {
        return direction >= 0.0 ? step : 0.0;
    }

    private static double positionError(double target, double position, boolean angular) {
        return angular ? Angle.wrap(target - position) : target - position;
    }

    private static boolean positive(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    /** Reports optimization progress and verification controls. */
    public void reportProgress(TunerContext context) {
        context.getTelemetry().addLine("Twiddle " + config.name + " PD tuning");
        context.getTelemetry().addData("Status", status);
        context.getTelemetry().addData("P / D", gains[0] + " / " + gains[1]);
        context.getTelemetry().addData("Best cost", bestCost);
        context.getTelemetry().addData("Trials", totalTrials);
        if (state == PDSState.TEST_READY) {
            context.getTelemetry().addLine("Press X to run a verification step.");
            context.getTelemetry().addLine("Press A to accept these gains.");
        }
        context.getTelemetry().update();
    }

    // endregion
}
