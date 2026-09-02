package tuning;

import java.util.function.Supplier;

import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

/**
 * Tunes the drive controller used by the follower using a {@link PDSRoutine}. The user can
 * manually tune the coefficients or run the automatic tuning routine.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Sohum Arora - 22985 Paraducks
 */
public class DrivePhase extends TuningPhase {
    private enum Coefficient { P, D, S }

    private static final double TRANSLATIONAL_KP_MIN = 0.01;
    private static final double TRANSLATIONAL_KP_MAX = 0.75;
    private static final double TRANSLATIONAL_KD_MIN = 0.0;
    private static final double TRANSLATIONAL_KD_MAX = 0.30;
    private static final double AUTOMATIC_TEST_DISTANCE = 24.0;
    private static final double POSITION_TOLERANCE = 0.75;
    private static final double SETTLING_VELOCITY = 1.0;
    private static final double AUTOMATIC_SAFETY_LIMIT = 36.0;

    private final Supplier<Path> testPath;
    private PDSRoutine routine;

    private Coefficient selected = Coefficient.P;
    private double target = 24.0;
    private double activeTestTarget;
    private boolean testPathQueued;
    private boolean testButtonHeld;
    private final ManualResponseMetrics manualMetrics = new ManualResponseMetrics();

    public DrivePhase(TunerContext context) {
        super(context);

        GeometryFactory factory = new GeometryFactory(context.getFollower())
                .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        testPath = () -> factory.path(
                context.getFollower().getPose(),
                context.getFollower().getPose().plus(factory.pose(target, 0.0, 0.0))
        ).interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();
    }

    @Override
    protected String getPhaseName() { return "Drive Controller"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void showPreRunInstructions() {
        context.getTelemetry().addLine(
                "Place the robot with at least 36 inches clear in front and behind it.");
        context.getTelemetry().addLine(
                "Automatic tuning calculates gains from the refined feedforward model, then " +
                        "checks settling, overshoot, error, and consistency in both directions.");
    }

    @Override
    protected void init() {
        positionRobotForSimulation(geometry.Pose.zero());
        testPathQueued = false;
        testButtonHeld = opMode.gamepad1.x;

        if (manualMode) {
            context.getFollower().enableControllers();
            context.getFollower().setDriveCoefficients(context.constants.translationalCoeffs);
            return;
        }

        PDSRoutine.Config config = PDSRoutine.Config.linear(
                "drive",
                TRANSLATIONAL_KP_MIN, TRANSLATIONAL_KP_MAX,
                TRANSLATIONAL_KD_MIN, TRANSLATIONAL_KD_MAX,
                AUTOMATIC_TEST_DISTANCE, POSITION_TOLERANCE,
                SETTLING_VELOCITY, AUTOMATIC_SAFETY_LIMIT
        );
        routine = new PDSRoutine(
                config,
                context.constants.translationalKV,
                context.constants.translationalKA,
                context.constants.translationalCoeffs.kS);
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
            context.constants.translationalCoeffs = routine.getCoefficients();
            context.getFollower().setDriveCoefficients(context.constants.translationalCoeffs);
            return true;
        }

        double position = context.getFollower().getPose().getX().getIn();
        double velocity = context.getFollower().getVelocity().getX().getIn();
        double command;
        try {
            command = routine.update(position, velocity);
        } catch (RuntimeException failure) {
            context.getFollower().stop();
            throw failure;
        }

        context.getFollower().getDrivetrain().moveWithVectors(command, 0.0, 0.0);
        routine.reportProgress(context);
        return false;
    }

    @Override
    protected boolean manualTuned() {
        if (manualMetrics.isActive()) {
            manualMetrics.sample(context.getFollower().getPose().getX().getIn(),
                    context.getFollower().getVelocity().getX().getIn(),
                    ManualResponseMetrics.maxMotorPower(
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
                context.constants.translationalCoeffs.kP = Math.max(
                        0.0, context.constants.translationalCoeffs.kP + change
                );
            } else if (selected == Coefficient.D) {
                context.constants.translationalCoeffs.kD = Math.max(
                        0.0, context.constants.translationalCoeffs.kD + change
                );
            } else if (selected == Coefficient.S) {
                context.constants.translationalCoeffs.kS = Math.max(
                        0.0, context.constants.translationalCoeffs.kS + change
                );
            }
            context.getFollower().setDriveCoefficients(context.constants.translationalCoeffs);
        }

        if (opMode.gamepad1.x && !testButtonHeld) { testPathQueued = true; }
        testButtonHeld = opMode.gamepad1.x;
        if (testPathQueued && !context.getFollower().isBusy()) {
            activeTestTarget = target;
            double start = context.getFollower().getPose().getX().getIn();
            manualMetrics.begin("manual_drive_response", start, start + target, 0.75, 1.0);
            context.getFollower().follow(testPath.get());
            target = -target;
            testPathQueued = false;
        }

        if (opMode.gamepad1.aWasPressed()) {
            manualMetrics.finish();
            return true;
        }

        addTunableValue("Drive P", context.constants.translationalCoeffs.kP,
                selected == Coefficient.P);
        addTunableValue("Drive D", context.constants.translationalCoeffs.kD,
                selected == Coefficient.D);
        addTunableValue("Drive S", context.constants.translationalCoeffs.kS,
                selected == Coefficient.S);
        context.getTelemetry().addData("Increment", number(increment));
        context.getTelemetry().addData("Active Test Target", number(activeTestTarget) + " in");
        context.getTelemetry().addData("Final error", number(manualMetrics.getFinalError()) + " in");
        if (context.isDebugMode()) { reportDetailedManualMetrics(); }
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("LB/RB: Select value to tune");
        context.getTelemetry().addLine("X: Run test path");
        context.getTelemetry().addLine("A: Save");
        context.getTelemetry().update();

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Drive P", number(context.constants.translationalCoeffs.kP));
        context.getTelemetry().addData("Drive D", number(context.constants.translationalCoeffs.kD));
        context.getTelemetry().addData("Drive S", number(context.constants.translationalCoeffs.kS));
    }

    private void reportDetailedManualMetrics() {
        context.getTelemetry().addData("Overshoot", manualMetrics.getOvershoot() + " in");
        context.getTelemetry().addData("Settling time",
                manualMetrics.getSettlingTime() + " s");
        context.getTelemetry().addData("RMS error", manualMetrics.getRmsError() + " in");
        context.getTelemetry().addData("Time-weighted squared error",
                manualMetrics.getTimeWeightedSquaredError());
        context.getTelemetry().addData("Peak velocity",
                manualMetrics.getPeakVelocity() + " in/s");
        context.getTelemetry().addData("Saturation", Math.round(
                manualMetrics.getSaturationFraction() * 1000.0) / 10.0 + "%");
        context.getTelemetry().addData("Response CSV", manualMetrics.getCsvPath());
    }
}
