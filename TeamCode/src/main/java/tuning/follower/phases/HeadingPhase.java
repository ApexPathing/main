package tuning.follower.phases;

import tuning.follower.TunerContext;
import tuning.follower.TuningPhase;

import geometry.Angle;
import geometry.AngleUnit;
import paths.builders.TurnBuilder;
import paths.movements.Turn;

/**
 * Tunes the heading controller used by the follower using a {@link PDSRoutine}. The user can
 * manually tune the coefficients or run the automatic tuning routine.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class HeadingPhase extends TuningPhase {
    private enum Coefficient { P, D, S }

    private static final double HEADING_KP_MIN = 0.10;
    private static final double HEADING_KP_MAX = 32.0;
    private static final double HEADING_KD_MIN = 0.0;
    private static final double HEADING_KD_MAX = 2.0;
    private static final double HEADING_INITIAL_KP = 0.80;
    private static final double HEADING_INITIAL_KD = 0.10;
    private static final double AUTOMATIC_TEST_ANGLE = Math.toRadians(60.0);
    private static final double POSITION_TOLERANCE = Math.toRadians(0.75);
    private static final double SETTLING_ANGULAR_VELOCITY = 0.10;
    private static final double AUTOMATIC_SAFETY_LIMIT = Math.toRadians(110.0);

    private PDSRoutine routine;

    private Coefficient selected = Coefficient.P;
    private double activeTestTarget = 0.0;
    private double nextTestTarget = 60.0;
    private boolean testTurnQueued = false;
    private Angle manualHeadingOrigin = Angle.fromRad(0.0);
    private final ManualResponseMetrics manualMetrics = new ManualResponseMetrics();

    public HeadingPhase(TunerContext context) {
        super(context);
    }

    @Override
    protected String getPhaseName() { return "Heading Controller"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void showPreRunInstructions() {
        context.getTelemetry().addLine(
                "Place the robot where it can rotate safely through a 60 degree out-and-back test.");
        context.getTelemetry().addLine(
                "Automatic tuning starts from generic gains, then " +
                        "checks settling, overshoot, error, and consistency in both directions.");
    }

    @Override
    protected void init() {
        positionRobotForSimulation(geometry.Pose.zero());
        context.getFollower().stop();

        if (manualMode) {
            context.getFollower().enableHeadingController();
            context.getFollower().disableDriveController();
            context.getFollower().setHeadingCoefficients(context.constants.angularCoeffs);
            manualHeadingOrigin = context.getFollower().getPose().getHeading();
            activeTestTarget = 0.0;
            nextTestTarget = 60.0;
            testTurnQueued = false;
            return;
        }

        PDSRoutine.Config config = PDSRoutine.Config.angular(
                "heading",
                HEADING_KP_MIN, HEADING_KP_MAX,
                HEADING_KD_MIN, HEADING_KD_MAX,
                AUTOMATIC_TEST_ANGLE, POSITION_TOLERANCE,
                SETTLING_ANGULAR_VELOCITY, AUTOMATIC_SAFETY_LIMIT
        );
        routine = new PDSRoutine(
                config,
                HEADING_INITIAL_KP,
                HEADING_INITIAL_KD,
                context.constants.angularCoeffs.kS);
        routine.start();
        context.getFollower().disableControllers();
    }

    @Override
    protected boolean autoTuned() {
        if (routine.isTestReady()) {
            if (opMode.gamepad1.xWasPressed()) { routine.requestTest(); }
            if (opMode.gamepad1.aWasPressed()) { routine.accept(); }
        }
        if (routine.isComplete()) {
            context.constants.angularCoeffs = routine.getCoefficients();
            context.getFollower().setHeadingCoefficients(context.constants.angularCoeffs);
            return true;
        }

        double position = context.getFollower().getPose().getHeading().getRad();
        double velocity = context.getFollower().getVelocity().getHeading(AngleUnit.RAD);
        double command;
        try {
            command = routine.update(position, velocity);
        } catch (RuntimeException failure) {
            context.getFollower().stop();
            throw failure;
        }

        context.getFollower().getDrivetrain().moveWithVectors(0.0, 0.0, command);
        routine.reportProgress(context);
        return false;
    }

    @Override
    protected boolean manualTuned() {
        if (manualMetrics.isActive()) {
            double position = manualHeadingOrigin.getShortestAngleTo(
                    context.getFollower().getPose().getHeading()).getRad();
            double velocity = context.getFollower().getVelocity().getHeading(AngleUnit.RAD);
            manualMetrics.sample(position, velocity, ManualResponseMetrics.maxMotorPower(
                    context.getFollower().getDrivetrain()));
            if (!context.getFollower().isBusy()) { manualMetrics.finish(); }
        }
        if (opMode.gamepad1.leftBumperWasPressed()) {
            selected = selected == Coefficient.P ? Coefficient.S :
                    Coefficient.values()[selected.ordinal() - 1];
        }
        if (opMode.gamepad1.rightBumperWasPressed()) {
            selected = selected == Coefficient.S ? Coefficient.P :
                    Coefficient.values()[selected.ordinal() + 1];
        }

        double change = manualChange();
        if (change != 0.0) {
            if (selected == Coefficient.P) {
                context.constants.angularCoeffs.kP = Math.max(
                        0.0, context.constants.angularCoeffs.kP + change
                );
            } else if (selected == Coefficient.D) {
                context.constants.angularCoeffs.kD = Math.max(
                        0.0, context.constants.angularCoeffs.kD + change
                );
            } else if (selected == Coefficient.S) {
                context.constants.angularCoeffs.kS = Math.max(
                        0.0, context.constants.angularCoeffs.kS + change
                );
            }
            context.getFollower().setHeadingCoefficients(context.constants.angularCoeffs);
        }

        if (opMode.gamepad1.xWasPressed()) {
            testTurnQueued = true;
        }
        if (testTurnQueued && !context.getFollower().isBusy()) {
            activeTestTarget = nextTestTarget;
            Turn testTurn = new TurnBuilder(context.getFollower().getPose())
                    .turnTo(Angle.fromRad(manualHeadingOrigin.getRad() +
                            Math.toRadians(activeTestTarget)))
                    .quickBuild();
            double start = manualHeadingOrigin.getShortestAngleTo(
                    context.getFollower().getPose().getHeading()).getRad();
            manualMetrics.begin("manual_heading_response", start,
                    Math.toRadians(activeTestTarget), Math.toRadians(2.5), 0.10);
            context.getFollower().follow(testTurn);
            nextTestTarget = nextManualTestTarget(activeTestTarget);
            testTurnQueued = false;
        }

        if (opMode.gamepad1.aWasPressed()) {
            manualMetrics.finish();
            return true;
        }

        addTunableValue("Heading P", context.constants.angularCoeffs.kP,
                selected == Coefficient.P);
        addTunableValue("Heading D", context.constants.angularCoeffs.kD,
                selected == Coefficient.D);
        addTunableValue("Heading S", context.constants.angularCoeffs.kS,
                selected == Coefficient.S);
        context.getTelemetry().addData("Increment", number(increment));
        context.getTelemetry().addData("Active Test Target", number(activeTestTarget) + " deg");
        context.getTelemetry().addData("Final error",
                number(Math.toDegrees(manualMetrics.getFinalError())) + " deg");
        if (context.isDebugMode()) { reportDetailedManualMetrics("rad", "rad/s"); }
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("LB/RB: select Value to tune");
        context.getTelemetry().addLine("X: Run test turn");
        context.getTelemetry().addLine("A: Save");
        context.getTelemetry().update();

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Heading P", number(context.constants.angularCoeffs.kP));
        context.getTelemetry().addData("Heading D", number(context.constants.angularCoeffs.kD));
        context.getTelemetry().addData("Heading S", number(context.constants.angularCoeffs.kS));
    }

    @Override
    protected boolean routineMotionActive() {
        if (manualMode) { return testTurnQueued || context.getFollower().isBusy(); }
        return routine != null && routine.getState() != PDSRoutine.PDSState.TEST_READY &&
                routine.getState() != PDSRoutine.PDSState.COMPLETE;
    }

    private void reportDetailedManualMetrics(String positionUnit, String velocityUnit) {
        context.getTelemetry().addData("Overshoot",
                manualMetrics.getOvershoot() + " " + positionUnit);
        context.getTelemetry().addData("Settling time",
                manualMetrics.getSettlingTime() + " s");
        context.getTelemetry().addData("RMS error",
                manualMetrics.getRmsError() + " " + positionUnit);
        context.getTelemetry().addData("Time-weighted squared error",
                manualMetrics.getTimeWeightedSquaredError());
        context.getTelemetry().addData("Peak velocity",
                manualMetrics.getPeakVelocity() + " " + velocityUnit);
        context.getTelemetry().addData("Saturation", Math.round(
                manualMetrics.getSaturationFraction() * 1000.0) / 10.0 + "%");
        context.getTelemetry().addData("Response CSV", manualMetrics.getCsvPath());
    }

    static double nextManualTestTarget(double completedTargetDegrees) {
        return Math.abs(completedTargetDegrees) < 1e-9 ? 60.0 : 0.0;
    }
}
