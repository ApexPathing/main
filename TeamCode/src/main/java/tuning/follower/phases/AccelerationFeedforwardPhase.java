package tuning.follower.phases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.Locale;

import core.Follower;
import geometry.AngleUnit;
import geometry.Pose;
import tuning.TuningCsvWriter;
import tuning.follower.TunerContext;
import tuning.follower.TuningPhase;

/** Tunes translational and angular kA with paired, fixed-profile velocity tracking trials. */
public final class AccelerationFeedforwardPhase extends TuningPhase {
    private static final int SEARCH_ROUNDS = 7;
    private static final double COMMAND_LIMIT = .85;
    private static final double CRUISE_SECONDS = .25;
    private static final double TRANSLATION_DISTANCE_IN = 60.0;
    private static final double TANK_ANGULAR_SWEEP_RAD = 4.5;
    private static final double HOLONOMIC_ANGULAR_SWEEP_RAD = 2.25;
    private static final double SCORE_START_FRACTION = .12;
    private static final double SCORE_END_FRACTION = .90;
    private static final double SETTLE_SECONDS = .60;
    private static final double MAX_SETTLE_SECONDS = 1.50;
    private static final int MIN_DIRECTION_SAMPLES = 20;

    private enum Axis { TRANSLATION, ANGULAR }
    private enum State { MANUAL_IDLE, SETTLING, RUNNING, DONE, FAILED }

    private final ElapsedTime timer = new ElapsedTime();
    private final double[] squaredErrorTime = new double[2];
    private final double[] signedErrorTime = new double[2];
    private final double[] scoredTime = new double[2];
    private final int[] samples = new int[2];
    private final int[] brakingSamples = new int[2];

    private Axis axis;
    private State state;
    private AccelerationSearch search;
    private TuningCsvWriter csv;
    private String csvPath = "Not started";
    private String result = "Not run";
    private double peakVelocity;
    private double targetAcceleration;
    private double accelerationSeconds;
    private double profileSeconds;
    private double searchUpper;
    private double candidateKA;
    private double pendingTranslationKA;
    private double pendingAngularKA;
    private double translationRms = Double.NaN;
    private double angularRms = Double.NaN;
    private int direction;
    private boolean confirmation;
    private double lastLoopTime;
    private double lastCommand;
    private boolean lastSampleEligible;
    private boolean saturated;
    private int trialNumber;

    public AccelerationFeedforwardPhase(TunerContext context) { super(context); }

    @Override protected String getPhaseName() { return "Acceleration Feedforward kA"; }
    @Override protected boolean manualTuneIsPossible() { return true; }
    @Override protected boolean autoTuneIsPossible() { return true; }

    @Override protected void showPreRunInstructions() {
        context.getTelemetry().addLine("Runs fixed forward/reverse and CCW/CW acceleration profiles.");
        context.getTelemetry().addLine("Translation profiles travel 60 inches each way; allow braking room.");
        context.getTelemetry().addLine("Angular sweeps are 50% longer; turning needs clear space.");
        context.getTelemetry().addLine("Automatic: A runs the entire search. Manual: adjust, then A runs.");
    }

    @Override protected void init() {
        Follower follower = context.getFollower();
        follower.stop();
        follower.disableControllers();
        pendingTranslationKA = context.constants.translationalKA;
        pendingAngularKA = context.constants.angularKA;
        translationRms = angularRms = Double.NaN;
        result = "Running";
        trialNumber = 0;
        openCsv();
        axis = Axis.TRANSLATION;
        if (!configureAxis()) {
            fail("Missing or invalid kS, kV, velocity limit, or acceleration limit.");
            return;
        }
        if (manualMode) {
            state = State.MANUAL_IDLE;
        } else {
            startAutomaticAxis();
        }
    }

    /** Always relinquishes direct motor control, including Driver Station Stop. */
    @Override public boolean run(LinearOpMode opMode) {
        try { return super.run(opMode); }
        finally {
            context.getFollower().stop();
            context.getFollower().enableControllers();
            closeCsv();
        }
    }

    private boolean configureAxis() {
        double velocityLimit = axis == Axis.TRANSLATION
                ? context.constants.forwardVelLimitIn : context.constants.angularVelLimitRad;
        double accelerationLimit = axis == Axis.TRANSLATION
                ? context.constants.forwardAccelLimitIn : context.constants.angularAccelLimitRad;
        double kS = axis == Axis.TRANSLATION
                ? context.constants.translationalFeedforwardKS
                : context.constants.angularFeedforwardKS;
        double kV = axis == Axis.TRANSLATION
                ? context.constants.translationalKV : context.constants.angularKV;
        boolean holonomic = context.getFollower().getDrivetrain().isHolonomic();
        double cap = axis == Axis.TRANSLATION ? 30.0 : holonomic ? 1.5 : 3.0;
        peakVelocity = Math.min(cap, Math.min(.50 * velocityLimit, .35 * accelerationLimit));
        // A trapezoid travels peakVelocity * (acceleration time + cruise time).
        // Put the extra travel into symmetric acceleration/braking ramps, with a short cruise.
        double travel = axis == Axis.TRANSLATION ? TRANSLATION_DISTANCE_IN
                : holonomic ? HOLONOMIC_ANGULAR_SWEEP_RAD : TANK_ANGULAR_SWEEP_RAD;
        accelerationSeconds = travel / peakVelocity - CRUISE_SECONDS;
        targetAcceleration = peakVelocity / accelerationSeconds;
        profileSeconds = 2 * accelerationSeconds + CRUISE_SECONDS;
        searchUpper = initialUpperBound(kS, kV, peakVelocity, targetAcceleration,
                accelerationLimit);
        return Double.isFinite(peakVelocity) && peakVelocity > 0.0
                && Double.isFinite(searchUpper) && searchUpper > 0.0;
    }

    static double safeUpperBound(double kS, double kV, double velocity,
                                 double acceleration) {
        if (!Double.isFinite(kS) || kS < 0.0 || !Double.isFinite(kV) || kV <= 0.0
                || !Double.isFinite(velocity) || velocity <= 0.0
                || !Double.isFinite(acceleration) || acceleration <= 0.0) {
            return Double.NaN;
        }
        double headroom = COMMAND_LIMIT - kS - kV * velocity;
        return headroom > 0.0 ? headroom / acceleration : Double.NaN;
    }

    static double initialUpperBound(double kS, double kV, double velocity,
                                    double acceleration, double measuredAccelerationLimit) {
        if (!Double.isFinite(measuredAccelerationLimit) || measuredAccelerationLimit <= 0.0) {
            return Double.NaN;
        }
        // The full-power acceleration test gives an inertia scale. Leave a factor of
        // two for measurement error, while retaining the hard command headroom bound.
        // Headroom alone gives enormous candidates on the slow angular test profile.
        return Math.min(safeUpperBound(kS, kV, velocity, acceleration),
                2.0 * (1.0 - kS) / measuredAccelerationLimit);
    }

    private void startAutomaticAxis() {
        search = new AccelerationSearch(searchUpper, SEARCH_ROUNDS);
        confirmation = false;
        scheduleTrial(search.current());
    }

    private void restartAutomatic() {
        closeCsv();
        openCsv();
        pendingTranslationKA = context.constants.translationalKA;
        pendingAngularKA = context.constants.angularKA;
        translationRms = angularRms = Double.NaN;
        result = "Running fresh search";
        trialNumber = 0;
        axis = Axis.TRANSLATION;
        if (!configureAxis()) {
            fail("Missing or invalid kS, kV, velocity limit, or acceleration limit.");
        } else {
            startAutomaticAxis();
        }
    }

    private void openCsv() {
        csv = TuningCsvWriter.open("acceleration_feedforward",
                "trial", "axis", "direction", "time_s", "candidate_kA",
                "target_velocity", "target_acceleration", "filtered_velocity",
                "raw_velocity", "command_power", "scored", "velocity_error");
        csvPath = csv.getPath();
    }

    private void scheduleTrial(double candidate) {
        candidateKA = Math.max(0.0, Math.min(searchUpper, candidate));
        direction = 0;
        saturated = false;
        Arrays.fill(squaredErrorTime, 0.0);
        Arrays.fill(signedErrorTime, 0.0);
        Arrays.fill(scoredTime, 0.0);
        Arrays.fill(samples, 0);
        Arrays.fill(brakingSamples, 0);
        trialNumber++;
        state = State.SETTLING;
        timer.reset();
        context.getFollower().stop();
    }

    private void updateRoutine() {
        if (state == State.SETTLING) {
            context.getFollower().stop();
            double speed = measuredVelocity(false);
            if (timer.seconds() >= SETTLE_SECONDS &&
                    (Math.abs(speed) <= stationaryThreshold()
                            || timer.seconds() >= MAX_SETTLE_SECONDS)) {
                beginDirection();
            }
            return;
        }
        if (state == State.RUNNING) { updateDirection(); }
    }

    private double stationaryThreshold() { return axis == Axis.TRANSLATION ? 1.0 : .1; }

    private void beginDirection() {
        lastLoopTime = 0.0;
        lastCommand = 0.0;
        lastSampleEligible = false;
        timer.reset();
        state = State.RUNNING;
    }

    private void updateDirection() {
        double elapsed = timer.seconds();
        double dt = elapsed - lastLoopTime;
        double filtered = measuredVelocity(false);
        double raw = measuredVelocity(true);
        double alignedFiltered = directionSign() * filtered;
        double alignedRaw = directionSign() * raw;
        // Velocity belongs to this timestamp; the preceding command belongs to the
        // interval that just elapsed. Scoring against its old target rewards lag.
        ProfilePoint currentTarget = profileAt(Math.min(elapsed, profileSeconds));
        double error = currentTarget.velocity - alignedFiltered;
        boolean usableDt = dt > 0.0 && dt <= .15;
        boolean scored = lastSampleEligible && isScoringTime(elapsed) && usableDt;
        if (scored && Double.isFinite(error)) {
            squaredErrorTime[direction] += error * error * dt;
            // A low kA undershoots acceleration but overshoots braking. Align both
            // errors with acceleration so the search moves in the same direction.
            signedErrorTime[direction] += error * Math.signum(currentTarget.acceleration) * dt;
            scoredTime[direction] += dt;
            samples[direction]++;
            if (currentTarget.acceleration < 0) { brakingSamples[direction]++; }
        }
        csv.writeRow(trialNumber, axis, direction == 0 ? "POSITIVE" : "NEGATIVE",
                elapsed, candidateKA, currentTarget.velocity, currentTarget.acceleration,
                alignedFiltered, alignedRaw, lastCommand,
                scored, error);

        if (!Double.isFinite(filtered) || !Double.isFinite(raw)) {
            fail("Localization returned a non-finite velocity.");
            return;
        }
        if (elapsed >= profileSeconds) {
            context.getFollower().stop();
            direction++;
            if (direction < 2) {
                state = State.SETTLING;
                timer.reset();
            } else {
                finishTrial();
            }
            return;
        }

        ProfilePoint point = profileAt(elapsed);
        double magnitude = feedforwardPower(point.velocity, point.acceleration, candidateKA);
        saturated |= !Double.isFinite(magnitude) || Math.abs(magnitude) > COMMAND_LIMIT;
        magnitude = Math.max(-COMMAND_LIMIT, Math.min(COMMAND_LIMIT, magnitude));
        double command = directionSign() * magnitude;
        context.getFollower().getDrivetrain().moveWithVectors(
                axis == Axis.TRANSLATION ? command : 0.0,
                0.0,
                axis == Axis.ANGULAR ? command : 0.0);
        lastLoopTime = elapsed;
        lastCommand = command;
        lastSampleEligible = isScoringTime(elapsed);
    }

    private boolean isScoringTime(double elapsed) {
        double rampTime = elapsed < accelerationSeconds + CRUISE_SECONDS
                ? elapsed : elapsed - accelerationSeconds - CRUISE_SECONDS;
        return rampTime >= SCORE_START_FRACTION * accelerationSeconds
                && rampTime <= SCORE_END_FRACTION * accelerationSeconds;
    }

    private ProfilePoint profileAt(double time) {
        if (time < accelerationSeconds) {
            return new ProfilePoint(targetAcceleration * time, targetAcceleration);
        }
        if (time < accelerationSeconds + CRUISE_SECONDS) {
            return new ProfilePoint(peakVelocity, 0.0);
        }
        double brakingTime = time - accelerationSeconds - CRUISE_SECONDS;
        return new ProfilePoint(Math.max(0.0, peakVelocity - targetAcceleration * brakingTime),
                -targetAcceleration);
    }

    private double feedforwardPower(double velocity, double acceleration, double kA) {
        double kS = axis == Axis.TRANSLATION
                ? context.constants.translationalFeedforwardKS
                : context.constants.angularFeedforwardKS;
        double kV = axis == Axis.TRANSLATION
                ? context.constants.translationalKV : context.constants.angularKV;
        return kS + kV * velocity + kA * acceleration;
    }

    private double measuredVelocity(boolean raw) {
        Pose velocity = raw ? context.getFollower().getRawVelocity()
                : context.getFollower().getVelocity();
        if (axis == Axis.ANGULAR) { return velocity.getHeading(AngleUnit.RAD); }
        Pose pose = context.getFollower().getPose();
        return velocity.getVec().rotate(pose.getHeading().times(-1)).getX().getIn();
    }

    private double directionSign() { return direction == 0 ? 1.0 : -1.0; }

    private void finishTrial() {
        context.getFollower().stop();
        TrialResult trial = TrialResult.of(squaredErrorTime, signedErrorTime,
                scoredTime, samples, saturated);
        for (int i = 0; i < 2; i++) {
            if (brakingSamples[i] < MIN_DIRECTION_SAMPLES
                    || samples[i] - brakingSamples[i] < MIN_DIRECTION_SAMPLES) {
                trial = new TrialResult(false, Double.NaN, Double.NaN);
            }
        }
        csv.writeRow(trialNumber, axis, "RESULT", "", candidateKA, "", "", "", "", "",
                trial.valid, trial.rms);
        if (!trial.valid) {
            if (manualMode) {
                result = "Invalid trial; press A to repeat after checking localization and space.";
                state = State.MANUAL_IDLE;
            } else {
                fail("Trial lacked enough valid unsaturated acceleration and braking samples.");
            }
            return;
        }

        if (manualMode) {
            acceptManualAxis(trial);
            return;
        }

        if (confirmation) {
            if (!confirmationAcceptable(search.bestRms(), trial.rms, peakVelocity)) {
                fail("Confirmation RMS worsened beyond the 25% / 2%-of-peak repeatability allowance.");
                return;
            }
            acceptAutomaticAxis(search.bestCandidate(), trial.rms);
            return;
        }

        search.record(trial.rms, trial.meanError);
        if (search.isComplete()) {
            confirmation = true;
            scheduleTrial(search.bestCandidate());
        } else {
            scheduleTrial(search.current());
        }
    }

    static boolean confirmationAcceptable(double bestRms, double confirmedRms, double peak) {
        if (!Double.isFinite(bestRms) || bestRms < 0.0
                || !Double.isFinite(confirmedRms) || confirmedRms < 0.0
                || !Double.isFinite(peak) || peak <= 0.0) { return false; }
        // A relative-only comparison becomes arbitrarily strict as the best score
        // approaches zero. Allow a small, axis-scaled repeatability floor as well.
        return confirmedRms <= bestRms + Math.max(.25 * bestRms, .02 * peak);
    }

    private void acceptAutomaticAxis(double value, double confirmedRms) {
        if (axis == Axis.TRANSLATION) {
            pendingTranslationKA = value;
            translationRms = confirmedRms;
            axis = Axis.ANGULAR;
            if (!configureAxis()) {
                fail("Angular kA could not be bounded from the tuned limits and kS/kV.");
                return;
            }
            startAutomaticAxis();
        } else {
            pendingAngularKA = value;
            angularRms = confirmedRms;
            applyPendingGains();
            result = "Both kA values passed paired-direction confirmation.";
            state = State.DONE;
            closeCsv();
        }
    }

    private void acceptManualAxis(TrialResult trial) {
        if (axis == Axis.TRANSLATION) {
            pendingTranslationKA = candidateKA;
            translationRms = trial.rms;
            axis = Axis.ANGULAR;
            if (!configureAxis()) {
                fail("Angular kA could not be bounded from the tuned limits and kS/kV.");
                return;
            }
            result = "Translation run complete; adjust angular kA and press A.";
            state = State.MANUAL_IDLE;
        } else {
            pendingAngularKA = candidateKA;
            angularRms = trial.rms;
            applyPendingGains();
            result = "Manual translation and angular runs complete.";
            state = State.DONE;
            closeCsv();
        }
    }

    private void applyPendingGains() {
        context.constants.translationalKA = pendingTranslationKA;
        context.constants.angularKA = pendingAngularKA;
        context.getFollower().setFeedforwardGains(
                context.constants.translationalKV, pendingTranslationKA,
                context.constants.angularKV, pendingAngularKA);
    }

    private void fail(String message) {
        context.getFollower().stop();
        result = message;
        state = State.FAILED;
        closeCsv();
    }

    @Override protected boolean autoTuned() {
        if (state == State.FAILED) {
            if (opMode.gamepad1.aWasPressed()) { restartAutomatic(); }
            showProgress();
            return false;
        }
        updateRoutine();
        showProgress();
        return state == State.DONE;
    }

    @Override protected boolean manualTuned() {
        if (state == State.MANUAL_IDLE) {
            double change = manualChange();
            if (change != 0.0) {
                if (axis == Axis.TRANSLATION) {
                    pendingTranslationKA = clampManual(pendingTranslationKA + change);
                } else {
                    pendingAngularKA = clampManual(pendingAngularKA + change);
                }
            }
            candidateKA = axis == Axis.TRANSLATION ? pendingTranslationKA : pendingAngularKA;
            if (opMode.gamepad1.aWasPressed()) { scheduleTrial(candidateKA); }
        } else {
            updateRoutine();
        }
        showManualProgress();
        return state == State.DONE || state == State.FAILED;
    }

    private double clampManual(double value) {
        return Math.max(0.0, Math.min(searchUpper, value));
    }

    private void showProgress() {
        context.getTelemetry().addLine(state == State.FAILED ? "Automatic kA tuning stopped."
                : "Automatic kA tuning in progress.");
        context.getTelemetry().addData("Axis", axis);
        context.getTelemetry().addData("Search", search == null ? "not started"
                : confirmation ? "confirmation"
                : (search.completed() + 1) + " / " + search.rounds());
        context.getTelemetry().addData("Candidate kA", number(candidateKA));
        context.getTelemetry().addData("Motion", motionDescription());
        context.getTelemetry().addData("Result", result);
        context.getTelemetry().addData("CSV", csvPath);
        if (state == State.FAILED) {
            context.getTelemetry().addLine("Press A to restart the complete search.");
        }
        context.getTelemetry().update();
    }

    private void showManualProgress() {
        addTunableValue(axis == Axis.TRANSLATION ? "Translation kA" : "Angular kA",
                axis == Axis.TRANSLATION ? pendingTranslationKA : pendingAngularKA, true);
        context.getTelemetry().addData("Increment", number(increment));
        context.getTelemetry().addData("Safe maximum", number(searchUpper));
        context.getTelemetry().addData("State", state == State.MANUAL_IDLE
                ? "IDLE - press A to run" : motionDescription());
        context.getTelemetry().addData("Result", result);
        context.getTelemetry().addLine("Dpad Up/Down: change kA");
        context.getTelemetry().addLine("Dpad Left/Right: change increment");
        context.getTelemetry().addLine("A: run this axis");
        context.getTelemetry().update();
    }

    private String motionDescription() {
        if (state == State.SETTLING) { return "settling before " + directionName(); }
        if (state == State.RUNNING) { return "running " + directionName(); }
        return state.name().toLowerCase(Locale.US);
    }

    private String directionName() {
        if (axis == Axis.TRANSLATION) { return direction == 0 ? "forward" : "reverse"; }
        return direction == 0 ? "counterclockwise" : "clockwise";
    }

    @Override protected void reportResults() {
        context.getTelemetry().addData("Translation kA / RMS",
                number(context.constants.translationalKA) + " / " + number(translationRms));
        context.getTelemetry().addData("Angular kA / RMS",
                number(context.constants.angularKA) + " / " + number(angularRms));
        context.getTelemetry().addData("Result", result);
        context.getTelemetry().addData("CSV", csvPath);
    }

    @Override protected boolean routineMotionActive() {
        return !manualMode || state != State.MANUAL_IDLE;
    }

    private void closeCsv() {
        if (csv != null) { csv.close(); csv = null; }
    }

    private static final class ProfilePoint {
        final double velocity;
        final double acceleration;
        ProfilePoint(double velocity, double acceleration) {
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    static final class TrialResult {
        final boolean valid;
        final double rms;
        final double meanError;

        private TrialResult(boolean valid, double rms, double meanError) {
            this.valid = valid;
            this.rms = rms;
            this.meanError = meanError;
        }

        static TrialResult of(double[] squared, double[] signed, double[] duration,
                              int[] samples, boolean saturated) {
            boolean valid = !saturated;
            double worstRms = 0.0;
            double signedTotal = 0.0;
            double durationTotal = 0.0;
            for (int i = 0; i < 2; i++) {
                valid &= samples[i] >= MIN_DIRECTION_SAMPLES && duration[i] > 0.0
                        && Double.isFinite(squared[i]) && Double.isFinite(signed[i]);
                if (duration[i] > 0.0) {
                    worstRms = Math.max(worstRms, Math.sqrt(squared[i] / duration[i]));
                    signedTotal += signed[i];
                    durationTotal += duration[i];
                }
            }
            double mean = durationTotal > 0.0 ? signedTotal / durationTotal : Double.NaN;
            valid &= Double.isFinite(worstRms) && Double.isFinite(mean);
            return new TrialResult(valid, valid ? worstRms : Double.NaN, mean);
        }
    }
}
