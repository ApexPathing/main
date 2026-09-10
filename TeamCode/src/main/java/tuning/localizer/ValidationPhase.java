package tuning.localizer;

import geometry.Pose;

/** Records a free-drive sanity check after geometry and filter calibration. */
public final class ValidationPhase extends TuningPhase {
    private static final long STOP_NANOS = 2_000_000_000L;
    private FilterMetrics metrics;
    private Pose startPose;
    private Pose endPose;
    private boolean stopping;
    private long stopStarted;

    public ValidationPhase(LocalizationTunerContext context) { super(context); }
    @Override protected String getName() { return "Validation drive"; }
    @Override protected void reset() { metrics = new FilterMetrics(); stopping = false; }
    @Override protected void showPrepare() {
        context.getTelemetry().addLine("Drive a loop with translation, turns, starts, and stops.");
        context.getTelemetry().addLine("Return to the marked start pose if practical.");
        context.getTelemetry().addLine("A: begin   B: cancel");
    }
    @Override protected void beginRecording() { startPose = context.getLocalizer().getPose(); }
    @Override protected boolean record() {
        Pose raw = context.getLocalizer().getRawVel();
        Pose filtered = context.getLocalizer().getVel();
        if (!stopping) {
            context.manualDrive(true);
            metrics.sample(raw, filtered, false, false,
                    context.getLocalizer().isLastMeasurementValid());
            context.getTelemetry().addLine("Drive the validation loop. Press A when finished.");
            if (opMode.gamepad1.aWasPressed()) {
                context.stop();
                stopping = true;
                stopStarted = System.nanoTime();
            }
            return false;
        }
        context.stop();
        metrics.sample(raw, filtered, true, true,
                context.getLocalizer().isLastMeasurementValid());
        context.getTelemetry().addLine("Measuring stopped drift...");
        return System.nanoTime() - stopStarted >= STOP_NANOS;
    }
    @Override protected void finishRecording() { endPose = context.getLocalizer().getPose(); }
    @Override protected void showReview() {
        Pose closure = endPose.minus(startPose);
        context.getTelemetry().addData("Start pose", startPose);
        context.getTelemetry().addData("End pose", endPose);
        context.getTelemetry().addData("Loop closure delta", closure);
        context.getTelemetry().addData("Stopped velocity RMS",
                context.formatNumber(metrics.stopResidualRms()));
        context.getTelemetry().addData("Invalid samples", metrics.getInvalidSamples());
        context.getTelemetry().addLine("Compare loop closure with the robot's physical return error.");
    }
    @Override protected boolean accept() { return context.acceptValidation(); }
}
