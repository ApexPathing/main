package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import geometry.Angle;

/** Finds the smallest motor command that produces measurable motion on one axis. */
final class StaticFrictionRoutine {
    private enum State { TESTING, SETTLING, COMPLETE }

    private static final double TEST_TIME_MS = 1_500.0;
    private static final double SETTLE_TIME_MS = 750.0;
    private static final double MAX_SEARCH_POWER = 0.75;

    private final double movementThreshold;
    private final boolean angular;
    private final ElapsedTime timer = new ElapsedTime();

    private BinarySearch search;
    private State state;
    private double trialStart = Double.NaN;

    /** Creates a bounded static-friction search for one axis. */
    StaticFrictionRoutine(double movementThreshold, boolean angular) {
        if (!Double.isFinite(movementThreshold) || movementThreshold <= 0.0) {
            throw new IllegalArgumentException("Movement threshold must be positive and finite");
        }
        this.movementThreshold = movementThreshold;
        this.angular = angular;
    }

    /** Resets and begins the search. */
    void start() {
        search = new BinarySearch(0.0, MAX_SEARCH_POWER, 0.01);
        state = State.TESTING;
        trialStart = Double.NaN;
        timer.reset();
    }

    /** Advances the search and returns the requested motor command. */
    double update(double absolutePosition) {
        if (search == null) {
            throw new IllegalStateException("Call start() before updating static friction");
        }
        if (!Double.isFinite(absolutePosition)) {
            throw new IllegalStateException("Static-friction tuning received a non-finite position");
        }
        if (state == State.COMPLETE) { return 0.0; }
        if (state == State.SETTLING) {
            if (timer.milliseconds() >= SETTLE_TIME_MS) {
                state = State.TESTING;
                trialStart = Double.NaN;
                timer.reset();
            }
            return 0.0;
        }

        if (Double.isNaN(trialStart)) {
            trialStart = absolutePosition;
            timer.reset();
        }
        double movement = relativePosition(absolutePosition, trialStart);
        boolean moved = Math.abs(movement) > movementThreshold;
        if (!moved && timer.milliseconds() < TEST_TIME_MS) {
            return search.current();
        }

        search.advance(moved
                ? BinarySearch.SearchDirection.LOWER
                : BinarySearch.SearchDirection.HIGHER);
        trialStart = Double.NaN;
        timer.reset();
        state = search.hasConverged() ? State.COMPLETE : State.SETTLING;
        return 0.0;
    }

    /** Returns whether the search has converged. */
    boolean isComplete() { return state == State.COMPLETE; }

    /** Returns the final static-friction command. */
    double getResult() {
        if (!isComplete()) {
            throw new IllegalStateException("Static-friction measurement is not complete");
        }
        return search.current();
    }

    /** Returns the command currently being tested. */
    double getCurrentGuess() { return search == null ? 0.0 : search.current(); }

    /** Measures linear or wrapped angular displacement from a trial origin. */
    private double relativePosition(double position, double origin) {
        double difference = position - origin;
        return angular ? Angle.wrap(difference) : difference;
    }
}
