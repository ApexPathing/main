package tuning.localizer.phases;

import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.LocalizerAdapter;
import tuning.localizer.TuningPhase;

import geometry.Pose;

/** Provides a live pose display while the driver moves the robot. */
public final class TestPhase extends TuningPhase {
    public TestPhase(LocalizationTunerContext context) { super(context); }

    @Override protected String getName() { return "Localization test"; }
    @Override protected void reset() { }

    @Override protected void showPrepare() {
        context.getTelemetry().addLine("Drive the robot and check its measured position.");
        context.getTelemetry().addLine("A: begin");
    }

    @Override protected void beginRecording() { }

    @Override protected boolean record() {
        context.manualDrive(true);
        Pose pose = context.getLocalizer().getPose();
        context.getTelemetry().addData("X", context.formatNumber(pose.getX().getIn()) + " in");
        context.getTelemetry().addData("Y", context.formatNumber(pose.getY().getIn()) + " in");
        context.getTelemetry().addData("Heading",
                context.formatNumber(pose.getHeading().getDeg()) + " deg");
        return false;
    }

    @Override protected void showReview() { }
    @Override protected boolean accept() { return false; }
}
