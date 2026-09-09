package tuning;

import geometry.AngleUnit;
import geometry.Pose;

/** Measures breakaway power for position correction, separately from moving feedforward kS. */
public final class StaticFrictionPhase extends TuningPhase {
    private enum Axis { HEADING, DRIVE, COMPLETE }
    private enum Stage { PROMPT, RUNNING }

    private static final double HEADING_MOVEMENT_THRESHOLD = 0.02;
    private static final double DRIVE_MOVEMENT_THRESHOLD = 0.25;

    private Axis axis;
    private Stage stage;
    private StaticFrictionRoutine routine;

    /** Creates the two-axis static-friction phase. */
    public StaticFrictionPhase(TunerContext context) { super(context); }

    /** Returns the label shown in the phase selector. */
    @Override
    protected String getPhaseName() { return "Breakaway Power"; }

    /** Breakaway power is measured automatically. */
    @Override
    protected boolean manualTuneIsPossible() { return false; }

    /** Breakaway power supports automatic measurement. */
    @Override
    protected boolean autoTuneIsPossible() { return true; }

    /** Displays the space needed by both measurements. */
    @Override
    protected void showPreRunInstructions() {
        context.getTelemetry().addLine(
                "This phase finds the minimum power needed to start turning and driving from rest.");
        context.getTelemetry().addLine(
                "Leave clear space for a small counterclockwise turn and forward movement.");
    }

    /** Stops the follower and selects heading as the first axis. */
    @Override
    protected void init() {
        context.getFollower().disableControllers();
        context.getFollower().stop();
        axis = Axis.HEADING;
        stage = Stage.PROMPT;
        routine = null;
    }

    /** Advances the active binary search and saves completed axes. */
    @Override
    protected boolean autoTuned() {
        if (axis == Axis.COMPLETE) { return true; }
        if (stage == Stage.PROMPT) {
            context.getFollower().stop();
            context.getTelemetry().addLine(axis == Axis.HEADING
                    ? "Next: minimum turning power. Keep the robot clear on all sides."
                    : "Next: minimum drive power. Point the front toward clear space.");
            context.getTelemetry().addLine("Press A when the robot is stationary and safe.");
            context.getTelemetry().update();
            if (opMode.gamepad1.aWasPressed()) { beginAxis(); }
            return false;
        }

        Pose pose = context.getFollower().getPose();
        double position = axis == Axis.HEADING
                ? pose.getHeading().getRad() : pose.getX().getIn();
        double command = routine.update(position);
        if (axis == Axis.HEADING) {
            context.getFollower().getDrivetrain().moveWithVectors(0.0, 0.0, command);
        } else {
            context.getFollower().getDrivetrain().moveWithVectors(command, 0.0, 0.0);
        }

        if (routine.isComplete()) {
            finishAxis();
        } else {
            context.getTelemetry().addLine(axis == Axis.HEADING
                    ? "Robot is measuring minimum turning power."
                    : "Robot is measuring minimum drive power.");
            if (context.isDebugMode()) {
                context.getTelemetry().addData("Axis", axis);
                context.getTelemetry().addData("Breakaway-power guess", routine.getCurrentGuess());
                context.getTelemetry().addData("Angular velocity",
                        context.getFollower().getVelocity().getHeading(AngleUnit.RAD));
            }
            context.getTelemetry().update();
        }
        return axis == Axis.COMPLETE;
    }

    /** Starts the selected axis measurement. */
    private void beginAxis() {
        positionRobotForSimulation(Pose.zero());
        routine = new StaticFrictionRoutine(
                axis == Axis.HEADING
                        ? HEADING_MOVEMENT_THRESHOLD : DRIVE_MOVEMENT_THRESHOLD,
                axis == Axis.HEADING);
        routine.start();
        stage = Stage.RUNNING;
    }

    /** Saves one measurement and selects the next axis. */
    private void finishAxis() {
        context.getFollower().stop();
        if (axis == Axis.HEADING) {
            context.constants.angularCoeffs.kS = routine.getResult();
            axis = Axis.DRIVE;
            stage = Stage.PROMPT;
        } else {
            context.constants.translationalCoeffs.kS = routine.getResult();
            axis = Axis.COMPLETE;
        }
    }

    /** Manual tuning is unavailable for this phase. */
    @Override
    protected boolean manualTuned() { return false; }

    /** Displays both measured breakaway powers. */
    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Heading breakaway power",
                number(context.constants.angularCoeffs.kS));
        context.getTelemetry().addData("Drive breakaway power",
                number(context.constants.translationalCoeffs.kS));
    }
}
