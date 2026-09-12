package tuning.follower.phases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import core.Follower;
import geometry.AngleUnit;
import geometry.Pose;
import geometry.Vector;
import tuning.TuningCsvWriter;
import tuning.follower.TunerContext;
import tuning.follower.TuningPhase;

/** Rising/falling power ramps separate moving friction, velocity drag, and inertial power. */
public final class FeedforwardTuner extends TuningPhase {
    // Tank fits account for inertia explicitly; holonomic drives retain the slow ramp fit.
    private double rampRate() {
        return .1;
    }
    private static final double MAX_POWER = .9;
    private static final double MAX_DISTANCE_IN = 96;
    private static final double MAX_TURN_RAD = 4 * Math.PI;
    private enum Axis { DRIVE, TURN }
    private enum Stage { READY, RAMP, REVIEW }

    private Axis axis = Axis.DRIVE;
    private Stage stage = Stage.READY;
    private final ElapsedTime timer = new ElapsedTime();
    private RampRegression regression;
    private Pose startPose;
    private double lastHeading, turnTravel, previousPower;
    private double downStarted = Double.NaN, peakPower;
    private double sampleTime, sampleVelocity, windowStartVelocity, windowTime, velocityIntegral, powerIntegral;
    private String result = "Not run";
    private String csvPath = "Not started";
    private TuningCsvWriter csv;
    private RampRegression.Fit driveFit;

    private boolean fitsInertia() { return !context.getFollower().getDrivetrain().isHolonomic(); }

    public FeedforwardTuner(TunerContext context) { super(context); }
    @Override protected String getPhaseName() { return "Feedforward kS / kV Ramp"; }
    @Override protected boolean manualTuneIsPossible() { return false; }
    @Override protected boolean autoTuneIsPossible() { return true; }
    @Override protected boolean manualTuned() { return false; }
    @Override protected boolean routineMotionActive() { return stage == Stage.RAMP; }

    @Override protected void showPreRunInstructions() {
        context.getTelemetry().addLine(fitsInertia()
                ? "Forward power rises then falls; a separate turning ramp follows."
                : "Slow forward ramp, then a separate counterclockwise turning ramp.");
        context.getTelemetry().addLine("Power rises by " + rampRate()
                + "/sec, up to 0.9. X stops a ramp for review.");
        context.getTelemetry().addLine("Clear 96 inches ahead and room to turn. Stop early before obstacles.");
        context.getTelemetry().addLine(fitsInertia()
                ? "Fits kS/kV while accounting for inertia; kA is validated in the next phase."
                : "Fits kS and kV together; kA is not tuned.");
    }

    @Override protected void init() {
        context.getFollower().stop();
        context.getFollower().enableControllers();
        axis = Axis.DRIVE;
        stage = Stage.READY;
        driveFit = null;
    }

    /** Always close evidence and release direct motor control, including Driver Station Stop. */
    @Override public boolean run(LinearOpMode opMode) {
        try { return super.run(opMode); }
        finally {
            context.getFollower().stop();
            context.getFollower().enableControllers();
            closeCsv();
        }
    }

    @Override protected boolean autoTuned() {
        if (stage == Stage.READY) {
            context.getTelemetry().addLine(axis == Axis.DRIVE
                    ? "Next: forward power ramp. Point toward clear space."
                    : "Next: counterclockwise power ramp. Clear space around the robot.");
            context.getTelemetry().addLine("Press A to start from rest. Sticks reposition while idle.");
            if (opMode.gamepad1.aWasPressed()) {
                Pose velocity = context.getFollower().getVelocity();
                if (velocity.getVec().getMag().getIn() > 1
                        || Math.abs(velocity.getHeading(AngleUnit.RAD)) > .1) {
                    result = "Wait until the robot is stationary.";
                } else { beginRamp(); }
            }
            context.getTelemetry().addData("Status", result);
            if (axis == Axis.TURN) { context.getFollower().setPose(Pose.zero()); }
        } else if (stage == Stage.RAMP) {
            sampleRamp();
        } else {
            if (opMode.gamepad1.xWasPressed()) {
                stage = Stage.READY;
                result = "Reposition, then start a fresh ramp.";
            } else {
                if (opMode.gamepad1.yWasPressed()) { regression.excludeWorst(); }
                if (opMode.gamepad1.bWasPressed()) { regression.restore(); }
                RampRegression.Fit fit = regression.fit();
                showFit(fit);
                context.getTelemetry().addLine("Y: exclude largest residual. B: restore excluded samples.");
                context.getTelemetry().addLine("X: discard/retry this ramp.");
                if (fit.valid(axis == Axis.DRIVE ? 2 : .2)) {
                    context.getTelemetry().addLine("Press A to accept this fit.");
                    if (opMode.gamepad1.aWasPressed()) {
                        writeReviewedSamples();
                        if (axis == Axis.DRIVE) {
                            driveFit = fit;
                            axis = Axis.TURN;
                            stage = Stage.READY;
                            result = "Forward fit accepted. Ready for turning ramp.";
                        } else {
                            applyFits(driveFit, fit);
                            result = "Forward and turning kS/kV accepted.";
                            return true;
                        }
                    }
                } else {
                    context.getTelemetry().addLine("Fit needs 20 moving samples, a speed range, positive kV,");
                    context.getTelemetry().addLine("nonnegative kS and R-squared >= 0.90. Retry or review outliers.");
                    if (fitsInertia()) {
                        context.getTelemetry().addLine("Tank fits also need both accelerating and braking samples.");
                    }
                }
            }
        }
        context.getTelemetry().update();
        return false;
    }

    private void beginRamp() {
        Follower follower = context.getFollower();
        follower.stop();
        follower.disableControllers();
        startPose = follower.getPose();
        lastHeading = startPose.getHeading().getRad();
        turnTravel = previousPower = 0;
        downStarted = Double.NaN;
        sampleTime = sampleVelocity = windowStartVelocity = windowTime = velocityIntegral = powerIntegral = 0;
        regression = new RampRegression(axis == Axis.DRIVE ? 1 : .1);
        closeCsv();
        csv = TuningCsvWriter.open("feedforward_ramp_" + axis.name().toLowerCase(java.util.Locale.US),
                "record", "axis", "time_s", "velocity", "applied_power", "included", "kS", "kV", "r_squared", "acceleration");
        csvPath = csv.getPath();
        timer.reset();
        stage = Stage.RAMP;
    }

    private void sampleRamp() {
        Follower follower = context.getFollower();
        Pose pose = follower.getPose();
        Pose velocity = fitsInertia() ? follower.getRawVelocity() : follower.getVelocity();
        double measured = axis == Axis.TURN ? velocity.getHeading(AngleUnit.RAD)
                : velocity.getVec().rotate(pose.getHeading().times(-1)).getX().getIn();
        double time = timer.seconds();
        // The measurement was produced by the PREVIOUS command, not the command we are about to send.
        double dt = time - sampleTime;
        if (!fitsInertia()) {
            boolean included = regression.add(measured, previousPower);
            csv.writeRow("raw", axis, time, measured, previousPower, included, "", "", "", "");
        } else if (Double.isFinite(measured) && dt > 0 && dt <= .15) {
            windowTime += dt;
            velocityIntegral += .5 * (sampleVelocity + measured) * dt;
            powerIntegral += previousPower * dt;
            if (windowTime >= .10) {
                // Integrate the motor equation over a window instead of differentiating
                // individual noisy samples. The power belongs to the preceding interval.
                double averageVelocity = velocityIntegral/windowTime;
                double averagePower = powerIntegral/windowTime;
                double acceleration = (measured-windowStartVelocity)/windowTime;
                boolean included = regression.add(averageVelocity, averagePower, acceleration);
                csv.writeRow("raw", axis, time, averageVelocity, averagePower, included,
                        "", "", "", acceleration);
                windowStartVelocity = measured;
                windowTime = velocityIntegral = powerIntegral = 0;
            }
        } else {
            windowStartVelocity = measured;
            windowTime = velocityIntegral = powerIntegral = 0;
        }
        sampleTime = time;
        sampleVelocity = measured;
        double heading = pose.getHeading().getRad();
        turnTravel += Math.abs(Math.atan2(Math.sin(heading - lastHeading), Math.cos(heading - lastHeading)));
        lastHeading = heading;
        double distance = pose.getVec().minus(startPose.getVec()).getMag().getIn();
        boolean travelLimit = axis == Axis.DRIVE ? distance >= MAX_DISTANCE_IN : turnTravel >= MAX_TURN_RAD;
        if (!Double.isFinite(measured) || !Double.isFinite(distance) || !Double.isFinite(turnTravel)) {
            stopRamp("Stopped: invalid localization measurement. Check localization and retry.");
        } else if (opMode.gamepad1.xWasPressed() || travelLimit
                || (!fitsInertia() && time >= MAX_POWER/rampRate())
                || (!Double.isNaN(downStarted) && time-downStarted >= peakPower/rampRate())) {
            stopRamp(travelLimit ? "Stopped at travel limit; review collected data." : "Ramp stopped; review collected data.");
        } else {
            // Reserve most of the travel allowance for the falling-power half.
            if (fitsInertia() && Double.isNaN(downStarted) && (rampRate()*time >= MAX_POWER
                    || (axis == Axis.DRIVE ? distance >= MAX_DISTANCE_IN/3
                            : turnTravel >= MAX_TURN_RAD/3))) {
                downStarted = time;
                peakPower = previousPower;
            }
            previousPower = Double.isNaN(downStarted) ? Math.min(MAX_POWER, rampRate()*time)
                    : Math.max(0, peakPower-rampRate()*(time-downStarted));
            follower.getDrivetrain().moveWithVectors(axis == Axis.DRIVE ? previousPower : 0,
                    0, axis == Axis.TURN ? previousPower : 0);
            // Fit the delivered command, including drivetrain power limits and write tolerance.
            previousPower = Math.max(Math.abs(follower.getDrivetrain().getLastFlPower()),
                    Math.abs(follower.getDrivetrain().getLastFrPower()));
            context.getTelemetry().addData("Ramp", axis);
            context.getTelemetry().addData("Power", number(previousPower));
            context.getTelemetry().addData("Measured velocity", number(measured));
            context.getTelemetry().addData("Moving samples", regression.samples.size());
            context.getTelemetry().addLine("X: stop now and review. Red STOP ends the OpMode.");
        }
    }

    private void stopRamp(String message) {
        context.getFollower().stop();
        context.getFollower().enableControllers();
        stage = Stage.REVIEW;
        result = message;
        closeCsv();
    }

    private void showFit(RampRegression.Fit fit) {
        context.getTelemetry().addData("Review", axis + " - " + result);
        context.getTelemetry().addData("kS / kV", number(fit.kS) + " / " + number(fit.kV));
        context.getTelemetry().addData("R-squared / included", number(fit.rSquared) + " / " + fit.count);
        context.getTelemetry().addData("Velocity span", number(fit.span));
        context.getTelemetry().addData("Units", axis == Axis.DRIVE ? "power, inches/sec" : "power, radians/sec");
        context.getTelemetry().addData("CSV", csvPath);
    }

    private void writeReviewedSamples() {
        TuningCsvWriter reviewed = TuningCsvWriter.open("feedforward_fit_" + axis.name().toLowerCase(java.util.Locale.US),
                "velocity", "power", "included", "kS", "kV", "r_squared", "acceleration", "inertia_estimate");
        RampRegression.Fit fit = regression.fit();
        for (RampRegression.Sample s : regression.samples) {
            reviewed.writeRow(s.velocity, s.power, !s.excluded, fit.kS, fit.kV, fit.rSquared,
                    s.acceleration, fit.kA);
        }
        reviewed.close();
    }

    private void applyFits(RampRegression.Fit drive, RampRegression.Fit turn) {
        context.constants.translationalFeedforwardKS = drive.kS;
        context.constants.angularFeedforwardKS = turn.kS;
        context.getFollower().setFeedforwardGains(drive.kV, context.constants.translationalKA,
                turn.kV, context.constants.angularKA);
    }

    private void closeCsv() { if (csv != null) { csv.close(); csv = null; } }

    @Override protected void reportResults() {
        context.getTelemetry().addData("Drive kS / kV", number(context.constants.translationalFeedforwardKS)
                + " / " + number(context.constants.translationalKV));
        context.getTelemetry().addData("Turn kS / kV", number(context.constants.angularFeedforwardKS)
                + " / " + number(context.constants.angularKV));
        context.getTelemetry().addLine("kA and position-controller breakaway settings are unchanged.");
        context.getTelemetry().addData("Result", result);
        context.getTelemetry().addData("Last ramp CSV", csvPath);
    }
}
