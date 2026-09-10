package tuning.localizer;

import geometry.Pose;
import localizers.BaseLocalizer;
import localizers.util.AdaptiveKalmanFilter;
import tuning.TuningCsvWriter;

/** Guides data collection and exposes either moving-average or adaptive-Kalman settings. */
public final class FilterPhase extends TuningPhase {
    private static final long STILL_NANOS = 3_000_000_000L;
    private static final long DRIVE_NANOS = 12_000_000_000L;
    private static final long STOP_NANOS = 3_000_000_000L;

    private BaseLocalizer.VelocityFilterMode selected =
            BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN;
    private int window = 7;
    private FilterSettings original;
    private FilterMetrics metrics;
    private TuningCsvWriter csv;
    private long started;

    public FilterPhase(LocalizationTunerContext context) { super(context); }
    @Override protected String getName() { return "Velocity and acceleration filter"; }

    @Override protected void reset() {
        selected = BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN;
        window = context.getLocalizer().getFilterWindowSize();
        metrics = new FilterMetrics();
    }

    @Override protected void showPrepare() {
        if (opMode.gamepad1.dpadLeftWasPressed() || opMode.gamepad1.dpadRightWasPressed()) {
            selected = selected == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN
                    ? BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE
                    : BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN;
        }
        if (selected == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
            if (opMode.gamepad1.dpadUpWasPressed()) { window = Math.min(50, window + 1); }
            if (opMode.gamepad1.dpadDownWasPressed()) { window = Math.max(1, window - 1); }
        }
        context.getTelemetry().addData("Selected", display(selected));
        if (selected == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
            context.getTelemetry().addData("Window", window);
            context.getTelemetry().addLine("Dpad Up/Down: adjust window");
        }
        context.getTelemetry().addLine("Dpad Left/Right: choose estimator");
        context.getTelemetry().addLine("Keep still, drive normally around the field, then stop.");
        context.getTelemetry().addLine("The tuner reports evidence; it does not choose a winner.");
        context.getTelemetry().addLine("A: begin   B: cancel");
    }

    @Override protected void beginRecording() {
        original = FilterSettings.capture(context.getLocalizer());
        context.getLocalizer().setVelocityFilterMode(selected);
        if (selected == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
            context.getLocalizer().setFilterWindow(window);
        } else {
            context.getLocalizer().setKalmanAutoTuning(true,
                    context.getDrivetrain().isHolonomic(), true);
        }
        metrics = new FilterMetrics();
        csv = TuningCsvWriter.open("localization_filter", "time_s", "stage", "valid",
                "raw_x", "raw_y", "raw_heading", "filtered_x", "filtered_y",
                "filtered_heading", "accel_x", "accel_y", "accel_heading");
        started = System.nanoTime();
    }

    @Override protected boolean record() {
        long elapsed = System.nanoTime() - started;
        boolean initialStill = elapsed < STILL_NANOS;
        boolean driving = elapsed >= STILL_NANOS && elapsed < STILL_NANOS + DRIVE_NANOS;
        boolean stopping = elapsed >= STILL_NANOS + DRIVE_NANOS;
        if (driving) { context.manualDrive(true); } else { context.stop(); }

        Pose raw = context.getLocalizer().getRawVel();
        Pose filtered = context.getLocalizer().getVel();
        Pose accel = context.getLocalizer().getAccel();
        metrics.sample(raw, filtered, initialStill, stopping,
                context.getLocalizer().isLastMeasurementValid());
        writeCsv(elapsed / 1e9, initialStill ? "STILL" : driving ? "DRIVE" : "STOP",
                raw, filtered, accel);
        context.getTelemetry().addData("Collection stage",
                initialStill ? "keep still" : driving ? "drive around field" : "release sticks and wait");
        context.getTelemetry().addData("Seconds", context.formatNumber(elapsed / 1e9));
        return elapsed >= STILL_NANOS + DRIVE_NANOS + STOP_NANOS;
    }

    @Override protected void finishRecording() {
        context.getLocalizer().setKalmanAutoTuning(false, false, false);
        if (csv != null) { csv.close(); }
    }

    @Override protected void showReview() {
        context.getTelemetry().addData("Estimator", display(selected));
        context.getTelemetry().addData("Raw still RMS", context.formatNumber(metrics.rawStationaryRms()));
        context.getTelemetry().addData("Filtered still RMS", context.formatNumber(metrics.filteredStationaryRms()));
        context.getTelemetry().addData("Stop residual RMS", context.formatNumber(metrics.stopResidualRms()));
        context.getTelemetry().addData("Peak raw / filtered",
                context.formatNumber(metrics.getPeakRaw()) + " / " + context.formatNumber(metrics.getPeakFiltered()));
        context.getTelemetry().addData("Invalid samples", metrics.getInvalidSamples());
        if (selected == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
            context.getTelemetry().addData("Window", context.getLocalizer().getFilterWindowSize());
        } else { showKalman(); }
        if (csv != null) {
            context.getTelemetry().addData("CSV", csv.getPath());
            if (csv.getError() != null) { context.getTelemetry().addData("CSV error", csv.getError()); }
        }
    }

    @Override protected boolean accept() { return context.acceptFilter(); }

    @Override protected void cancel() {
        if (csv != null) { csv.close(); }
        if (original != null) { original.apply(context.getLocalizer()); }
    }

    private void showKalman() {
        addTuning("X", context.getLocalizer().getXKalmanTuning());
        addTuning("Y", context.getLocalizer().getYKalmanTuning());
        addTuning("Heading", context.getLocalizer().getHeadingKalmanTuning());
    }

    private void addTuning(String axis, AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (tuning == null) { return; }
        context.getTelemetry().addData(axis + " R / Q",
                context.formatNumber(tuning.measurementVariance) + " / "
                        + context.formatNumber(tuning.processVariance));
    }

    private void writeCsv(double time, String stage, Pose raw, Pose filtered, Pose accel) {
        if (csv == null) { return; }
        csv.writeRow(time, stage, context.getLocalizer().isLastMeasurementValid(),
                raw.getX().getIn(), raw.getY().getIn(), raw.getHeading().getRad(),
                filtered.getX().getIn(), filtered.getY().getIn(), filtered.getHeading().getRad(),
                accel.getX().getIn(), accel.getY().getIn(), accel.getHeading().getRad());
    }

    private static String display(BaseLocalizer.VelocityFilterMode mode) {
        return mode == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN
                ? "Adaptive Kalman" : "Moving average";
    }
}
