package tuning.localizer;

import geometry.Pose;

/** Confirms that the configured localizer produces fresh, finite measurements. */
public final class HardwarePhase extends TuningPhase {
    private static final long RECORD_NANOS = 3_000_000_000L;
    private long started;
    private int validSamples;
    private int invalidSamples;
    private Pose startPose = Pose.zero();
    private Pose endPose = Pose.zero();

    public HardwarePhase(LocalizationTunerContext context) { super(context); }
    @Override protected String getName() { return "Hardware check"; }
    @Override protected void reset() { validSamples = invalidSamples = 0; }
    @Override protected void showPrepare() {
        context.getTelemetry().addLine("Lift the drive wheels or leave clear floor around the robot.");
        context.getTelemetry().addLine("A: monitor sensors for 3 seconds   B: cancel");
    }
    @Override protected void beginRecording() {
        started = System.nanoTime();
        startPose = context.getLocalizer().getPose();
    }
    @Override protected boolean record() {
        if (context.getLocalizer().isLastMeasurementValid()) { validSamples++; }
        else { invalidSamples++; }
        context.manualDrive(true);
        context.getTelemetry().addData("Valid samples", validSamples);
        context.getTelemetry().addData("Invalid samples", invalidSamples);
        return System.nanoTime() - started >= RECORD_NANOS;
    }
    @Override protected void finishRecording() { endPose = context.getLocalizer().getPose(); }
    @Override protected void showReview() {
        context.getTelemetry().addData("Valid samples", validSamples);
        context.getTelemetry().addData("Invalid samples", invalidSamples);
        context.getTelemetry().addData("Pose start", startPose);
        context.getTelemetry().addData("Pose end", endPose);
    }
    @Override protected boolean accept() { return true; }
}
