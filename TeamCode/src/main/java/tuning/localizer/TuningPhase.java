package tuning.localizer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Common prepare, record, and confirmation lifecycle for localization-tuner procedures.
 */
public abstract class TuningPhase {
    protected enum State { PREPARE, RECORD, REVIEW, ERROR }

    protected final LocalizationTunerContext context;
    protected LinearOpMode opMode;
    private State state;
    private String error = "";

    protected TuningPhase(LocalizationTunerContext context) { this.context = context; }

    /** Runs until the result is accepted or the OpMode stops. */
    public final boolean run(LinearOpMode opMode) {
        this.opMode = opMode;
        state = State.PREPARE;
        reset();
        while (opMode.opModeIsActive()) {
            context.update();
            context.beginFrame();
            context.getTelemetry().addLine(getName());

            if (context.isDriveTransitioning()) {
                context.stop();
                context.getTelemetry().addLine("Waiting for drivetrain mode to settle...");
                context.getTelemetry().update();
                context.pauseLoop();
                continue;
            }

            if (state == State.PREPARE) {
                context.stop();
                showPrepare();
                if (opMode.gamepad1.aWasPressed()) {
                    error = "";
                    beginRecording();
                    state = State.RECORD;
                }
            } else if (state == State.RECORD) {
                try {
                    if (record()) {
                        context.stop();
                        finishRecording();
                        state = State.REVIEW;
                    }
                } catch (RuntimeException failure) {
                    context.stop();
                    error = failure.getMessage();
                    state = State.ERROR;
                }
            } else if (state == State.REVIEW) {
                context.stop();
                showReview();
                if (!error.isEmpty()) { context.getTelemetry().addData("Save error", error); }
                context.getTelemetry().addLine("A: save");
                if (opMode.gamepad1.aWasPressed()) {
                    if (accept()) { return true; }
                    error = context.getLastSaveError();
                }
            } else {
                context.stop();
                context.getTelemetry().addData("Could not record", error);
                context.getTelemetry().addLine("Stop the OpMode and run the phase again.");
            }
            context.getTelemetry().update();
            context.pauseLoop();
        }
        context.stop();
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
}
