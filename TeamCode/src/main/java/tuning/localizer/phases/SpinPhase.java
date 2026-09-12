package tuning.localizer.phases;

import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.LocalizerAdapter;
import tuning.localizer.TuningPhase;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Collections;

import geometry.Angle;
import localizers.util.LowPassFilter;

/** Calibrates rotational geometry from five counterclockwise rotations. */
public final class SpinPhase extends TuningPhase {
    static final double TURN_VELOCITY_RAD_PER_SECOND = Math.toRadians(45);
    static final double POWER_RAMP_PER_SECOND = 0.03;
    private static final double MAX_TURN_POWER = 0.50;
    private static final double MIN_MANUAL_ANGLE_DEGREES = 180.0;
    private static final double TARGET_ROTATIONS = 5.0;
    private static final double TARGET_ANGLE_RADIANS = TARGET_ROTATIONS * 2.0 * Math.PI;
    private static final long SETTLE_NANOS = 400_000_000L;

    private enum Stage { SPIN, SETTLE, ENTER_ANGLE }

    private final RelativeImuAccumulator imuAngle = new RelativeImuAccumulator();
    private IMU imu;
    private boolean useImu = true;
    private CalibrationSnapshot start;
    private double reportedAngle;
    private double previousHeading;
    private double manualAngleDegrees;
    private double manualIncrementDegrees;
    private double turnPower;
    private double measuredAngularVelocity;
    private final LowPassFilter angularVelocityFilter = new LowPassFilter();
    private boolean turnVelocityReached;
    private long controlLastUpdateNanos;
    private long stageStarted;
    private Stage stage;
    private CalibrationCandidate candidate;

    public SpinPhase(LocalizationTunerContext context) { super(context); }
    @Override protected String getName() { return "Rotation geometry"; }

    @Override protected void reset() {
        candidate = null;
        stage = Stage.SPIN;
        manualAngleDegrees = 360.0;
        manualIncrementDegrees = 15.0;
        imuAngle.clearAxis();
        try { imu = opMode == null ? null : opMode.hardwareMap.get(IMU.class, "imu"); }
        catch (RuntimeException ignored) { imu = null; }
        useImu = imu != null;
    }

    @Override protected void showPrepare() {
        if ((opMode.gamepad1.dpadLeftWasPressed() || opMode.gamepad1.dpadRightWasPressed())
                && imu != null) {
            useImu = !useImu;
        }
        context.getTelemetry().addData("Reference", useImu ? "IMU" : "Manual");
        if (imu != null) {
            context.getTelemetry().addLine("Dpad Left/Right: change reference");
        }
        context.getTelemetry().addLine("Robot spins left at 45 deg/s for 5 rotations.");
        if (!useImu) { context.getTelemetry().addLine("Measure the total rotation."); }
        context.getTelemetry().addLine("A: begin");
    }

    @Override protected void beginRecording() {
        context.stop();
        start = context.getAdapter().snapshot(context.getLocalizer());
        previousHeading = start.pose.getHeading().getRad();
        reportedAngle = 0.0;
        turnPower = 0.0;
        measuredAngularVelocity = 0.0;
        turnVelocityReached = false;
        stage = Stage.SPIN;
        if (useImu) { imuAngle.resetMeasurement(readQuaternion()); }
        stageStarted = System.nanoTime();
        controlLastUpdateNanos = stageStarted;
        angularVelocityFilter.reset();
    }

    @Override protected boolean record() {
        updateReportedAngle();
        if (useImu && stage != Stage.ENTER_ANGLE) { imuAngle.update(readQuaternion()); }

        if (stage == Stage.SPIN) {
            double measuredAngle = useImu ? imuAngle.getAngleRad() : reportedAngle;
            context.getTelemetry().addData("Rotations",
                    context.formatNumber(Math.min(TARGET_ROTATIONS,
                            Math.abs(measuredAngle) / (2.0 * Math.PI)))
                            + " / " + context.formatNumber(TARGET_ROTATIONS));
            if (Math.abs(measuredAngle) >= TARGET_ANGLE_RADIANS) {
                context.stop();
                stage = Stage.SETTLE;
                stageStarted = System.nanoTime();
                return false;
            }

            long now = System.nanoTime();
            double dt = Math.min(0.10, Math.max(1e-3,
                    (now - controlLastUpdateNanos) * 1e-9));
            measuredAngularVelocity = angularVelocityFilter.update(measuredAngle).rate();
            if (!turnVelocityReached) {
                if (Math.abs(measuredAngularVelocity) >= TURN_VELOCITY_RAD_PER_SECOND) {
                    turnVelocityReached = true;
                } else {
                    turnPower = rampTurnPower(turnPower, dt);
                }
            }
            context.getDrivetrain().moveWithVectors(0.0, 0.0, turnPower);
            controlLastUpdateNanos = now;
            context.getTelemetry().addData("Turn velocity",
                    context.formatNumber(Math.toDegrees(measuredAngularVelocity))
                            + " / 45 deg/s");
            context.getTelemetry().addData("Turn power", context.formatNumber(turnPower));
            context.getTelemetry().addData("Power", turnVelocityReached ? "latched" : "ramping");
            return false;
        }

        context.stop();
        if (stage == Stage.SETTLE) {
            context.getTelemetry().addLine("Stopping...");
            if (System.nanoTime() - stageStarted < SETTLE_NANOS) { return false; }
            if (useImu) {
                finishTrial(imuAngle.getAngleRad());
                return true;
            }
            manualAngleDegrees = Math.max(MIN_MANUAL_ANGLE_DEGREES,
                    Math.abs(Math.toDegrees(reportedAngle)));
            stage = Stage.ENTER_ANGLE;
        }

        editManualAngle();
        context.getTelemetry().addData("Measured rotation",
                context.formatNumber(manualAngleDegrees) + " deg");
        context.getTelemetry().addData("Increment",
                context.formatNumber(manualIncrementDegrees) + " deg");
        context.getTelemetry().addLine("Up/Down: edit   Left/Right: increment");
        context.getTelemetry().addLine("A: confirm");
        if (opMode.gamepad1.aWasPressed()) {
            finishTrial(Math.toRadians(manualAngleDegrees));
            return true;
        }
        return false;
    }

    @Override protected void showReview() {
        for (String metric : candidate.getMetrics()) { context.getTelemetry().addLine(metric); }
        if (useImu) {
            context.getTelemetry().addData("IMU axis", String.format(java.util.Locale.US,
                    "%.2f, %.2f, %.2f", imuAngle.getSpinAxisX(), imuAngle.getSpinAxisY(),
                    imuAngle.getSpinAxisZ()));
        }
    }

    @Override protected boolean accept() { return context.acceptGeometry("ROTATION", candidate); }

    private void finishTrial(double referenceAngle) {
        CalibrationSnapshot end = context.getAdapter().snapshot(context.getLocalizer());
        SpinTrial trial = new SpinTrial(start, end, referenceAngle, reportedAngle);
        candidate = context.getAdapter().fitSpin(context.getConfig(),
                Collections.singletonList(trial));
    }

    private void editManualAngle() {
        if (opMode.gamepad1.dpadLeftWasPressed()) {
            manualIncrementDegrees = Math.max(1.0, manualIncrementDegrees / 5.0);
        }
        if (opMode.gamepad1.dpadRightWasPressed()) {
            manualIncrementDegrees = Math.min(90.0, manualIncrementDegrees * 5.0);
        }
        if (opMode.gamepad1.dpadUpWasPressed()) {
            manualAngleDegrees += manualIncrementDegrees;
        }
        if (opMode.gamepad1.dpadDownWasPressed()) {
            manualAngleDegrees = Math.max(MIN_MANUAL_ANGLE_DEGREES,
                    manualAngleDegrees - manualIncrementDegrees);
        }
    }

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

    /** Raises power at a fixed rate during the one-time velocity-discovery ramp. */
    static double rampTurnPower(double power, double dt) {
        return Math.max(0.0, Math.min(MAX_TURN_POWER,
                power + POWER_RAMP_PER_SECOND * Math.max(0.0, dt)));
    }

}
