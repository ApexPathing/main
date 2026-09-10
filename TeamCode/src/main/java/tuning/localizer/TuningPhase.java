package tuning.localizer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Common Prepare, Record, Review lifecycle for localization-tuner procedures.
 * A accepts, B cancels, and X repeats a completed recording.
 */
public abstract class TuningPhase {
    protected enum State { PREPARE, RECORD, REVIEW }

    protected final LocalizationTunerContext context;
    protected LinearOpMode opMode;
    private State state;
    private String error = "";

    protected TuningPhase(LocalizationTunerContext context) { this.context = context; }

    /** Runs until the result is accepted, canceled, or the OpMode stops. */
    public final boolean run(LinearOpMode opMode) {
        this.opMode = opMode;
        state = State.PREPARE;
        reset();
        while (opMode.opModeIsActive()) {
            context.updateDebugMode(false);
            context.update();
            context.getTelemetry().clearAll();
            context.addInterfaceHeader();
            context.getTelemetry().addData("Phase", getName());
            context.getTelemetry().addData("Step", state);

            if (context.isDriveTransitioning()) {
                context.stop();
                context.getTelemetry().addLine("Waiting for drivetrain mode to settle...");
                context.getTelemetry().update();
                opMode.sleep(20);
                continue;
            }

            if (state == State.PREPARE) {
                context.stop();
                showPrepare();
                if (opMode.gamepad1.aWasPressed()) {
                    error = "";
                    beginRecording();
                    state = State.RECORD;
                } else if (opMode.gamepad1.bWasPressed()) {
                    context.stop();
                    cancel();
                    return false;
                }
            } else if (state == State.RECORD) {
                if (opMode.gamepad1.bWasPressed()) {
                    context.stop();
                    cancel();
                    return false;
                }
                try {
                    if (record()) {
                        context.stop();
                        finishRecording();
                        state = State.REVIEW;
                    }
                } catch (RuntimeException failure) {
                    context.stop();
                    error = failure.getMessage();
                    state = State.PREPARE;
                    reset();
                }
            } else {
                context.stop();
                showReview();
                if (!error.isEmpty()) { context.getTelemetry().addData("Save error", error); }
                context.getTelemetry().addLine("A: accept and save   X: record again   B: cancel");
                if (opMode.gamepad1.aWasPressed()) {
                    if (accept()) { return true; }
                    error = context.getLastSaveError();
                } else if (opMode.gamepad1.xWasPressed()) {
                    cancel();
                    reset();
                    state = State.PREPARE;
                } else if (opMode.gamepad1.bWasPressed()) {
                    cancel();
                    return false;
                }
            }
            if (!error.isEmpty() && state == State.PREPARE) {
                context.getTelemetry().addData("Could not record", error);
            }
            context.getTelemetry().update();
            opMode.sleep(20);
        }
        context.stop();
        cancel();
        return false;
    }

    protected abstract String getName();
    protected abstract void reset();
    protected abstract void showPrepare();
    protected abstract void beginRecording();
    protected abstract boolean record();
    protected void finishRecording() { }
    protected abstract void showReview();
    protected abstract boolean accept();
    protected void cancel() { }
}
