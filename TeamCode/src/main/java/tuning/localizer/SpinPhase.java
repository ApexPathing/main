package tuning.localizer;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.ArrayList;
import java.util.List;

import geometry.Angle;

/** Calibrates rotational geometry from paired counterclockwise and clockwise spins. */
public final class SpinPhase extends TuningPhase {
    private static final double TARGET_ANGLE = 4.0 * Math.PI;
    private static final double AUTO_POWER = 0.50;
    private static final long TIMEOUT_NANOS = 15_000_000_000L;
    private static final long SETTLE_NANOS = 400_000_000L;

    private final RelativeImuAccumulator imuAngle = new RelativeImuAccumulator();
    private final List<SpinTrial> trials = new ArrayList<>();
    private IMU imu;
    private boolean useImu = true;
    private int direction;
    private CalibrationSnapshot start;
    private double reportedAngle;
    private double previousHeading;
    private long stageStarted;
    private boolean settling;
    private CalibrationCandidate candidate;

    public SpinPhase(LocalizationTunerContext context) { super(context); }
    @Override protected String getName() { return "Rotation geometry"; }

    @Override protected void reset() {
        trials.clear();
        candidate = null;
        direction = 1;
        settling = false;
        imuAngle.clearAxis();
        try { imu = opMode == null ? null : opMode.hardwareMap.get(IMU.class, "imu"); }
        catch (RuntimeException ignored) { imu = null; }
        useImu = imu != null;
    }

    @Override protected void showPrepare() {
        if (opMode.gamepad1.dpadLeftWasPressed() || opMode.gamepad1.dpadRightWasPressed()) {
            if (imu != null) { useImu = !useImu; }
        }
        context.getTelemetry().addData("Rotation reference", useImu ? "IMU (imu)" : "Manual");
        if (imu == null) { context.getTelemetry().addLine("Device 'imu' unavailable; manual reference selected."); }
        else { context.getTelemetry().addLine("Dpad Left/Right: choose IMU or manual reference."); }
        context.getTelemetry().addLine("The phase records two turns CCW and two turns CW.");
        context.getTelemetry().addLine(useImu
                ? "The robot turns automatically; keep a clear radius."
                : "Use only Right Stick X. Translation is disabled; press A after exactly two turns.");
        context.getTelemetry().addLine("A: begin   B: cancel");
    }

    @Override protected void beginRecording() { beginTrial(); }

    @Override protected boolean record() {
        updateReportedAngle();
        double reference = useImu ? updateImu() : direction * TARGET_ANGLE;
        context.getTelemetry().addData("Direction", direction > 0 ? "counterclockwise" : "clockwise");
        context.getTelemetry().addData("Reported rotation",
                context.formatNumber(reportedAngle) + " rad");
        if (useImu) {
            context.getTelemetry().addData("IMU rotation",
                    context.formatNumber(imuAngle.getAngleRad()) + " rad");
        }

        if (settling) {
            context.stop();
            if (System.nanoTime() - stageStarted < SETTLE_NANOS) { return false; }
            finishTrial(reference);
            if (direction > 0) {
                direction = -1;
                beginTrial();
                return false;
            }
            candidate = context.getAdapter().fitSpin(context.getConfig(), trials);
            return true;
        }

        if (useImu) {
            context.getDrivetrain().moveWithVectors(0.0, 0.0, AUTO_POWER * direction);
            if (Math.abs(reference) >= TARGET_ANGLE) { startSettling(); }
            else if (System.nanoTime() - stageStarted >= TIMEOUT_NANOS) {
                throw new IllegalStateException("IMU spin timed out; retry with manual reference");
            }
        } else {
            context.manualDrive(false);
            if (opMode.gamepad1.aWasPressed()) { startSettling(); }
        }
        return false;
    }

    @Override protected void showReview() {
        for (String metric : candidate.getMetrics()) { context.getTelemetry().addLine(metric); }
        if (useImu) {
            context.getTelemetry().addData("IMU off-axis RMS",
                    context.formatNumber(imuAngle.getRmsOffAxisRad()) + " rad");
        }
    }

    @Override protected boolean accept() { return context.acceptGeometry("ROTATION", candidate); }

    private void beginTrial() {
        context.stop();
        start = context.getAdapter().snapshot(context.getLocalizer());
        previousHeading = start.pose.getHeading().getRad();
        reportedAngle = 0.0;
        settling = false;
        stageStarted = System.nanoTime();
        if (useImu) { imuAngle.resetMeasurement(readQuaternion()); }
    }

    private void startSettling() {
        context.stop();
        settling = true;
        stageStarted = System.nanoTime();
    }

    private void finishTrial(double reference) {
        CalibrationSnapshot end = context.getAdapter().snapshot(context.getLocalizer());
        trials.add(new SpinTrial(start, end, useImu ? reference : direction * TARGET_ANGLE,
                reportedAngle));
    }

    private double updateImu() { return imuAngle.update(readQuaternion()); }

    private Quaternion readQuaternion() {
        try { return imu.getRobotOrientationAsQuaternion(); }
        catch (RuntimeException failure) {
            throw new IllegalStateException("Device 'imu' stopped returning orientation", failure);
        }
    }

    private void updateReportedAngle() {
        double heading = context.getLocalizer().getPose().getHeading().getRad();
        reportedAngle += Angle.wrap(heading - previousHeading);
        previousHeading = heading;
    }
}
