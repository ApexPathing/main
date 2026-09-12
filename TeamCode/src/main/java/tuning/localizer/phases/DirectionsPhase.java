package tuning.localizer.phases;

import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.LocalizerAdapter;
import tuning.localizer.TuningPhase;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.ArrayList;
import java.util.List;

import geometry.Angle;
import geometry.Pose;
import geometry.Vector;

/**
 * Applies short robot-centric pulses and reports whether each localizer axis has the expected sign.
 * Holonomic drivetrains check forward, strafe, and turn; tank drivetrains omit strafe.
 */
public final class DirectionsPhase extends TuningPhase {
    private static final double POWER = 0.50;
    private static final long DRIVE_NANOS = 500_000_000L;
    private static final long SETTLE_NANOS = 300_000_000L;
    private static final double MIN_TRANSLATION_INCHES = 0.25;
    private static final double MIN_ROTATION_RADIANS = Math.toRadians(5.0);

    private enum Motion { FORWARD, STRAFE, TURN }
    private enum Stage { DRIVE, SETTLE }

    private final List<Motion> motions = new ArrayList<>();
    private final List<Response> responses = new ArrayList<>();
    private final List<DirectionTrial> trials = new ArrayList<>();
    private final RelativeImuAccumulator imuRotation = new RelativeImuAccumulator();
    private IMU imu;
    private String imuError = "";
    private int motionIndex;
    private int invalidSamples;
    private long stageStarted;
    private Stage stage;
    private CalibrationSnapshot start;
    private CalibrationCandidate candidate;

    public DirectionsPhase(LocalizationTunerContext context) { super(context); }

    @Override protected String getName() { return "Direction check"; }

    @Override protected void reset() {
        motions.clear();
        motions.add(Motion.FORWARD);
        if (context.getDrivetrain().isHolonomic()) { motions.add(Motion.STRAFE); }
        motions.add(Motion.TURN);
        responses.clear();
        trials.clear();
        candidate = null;
        motionIndex = 0;
        invalidSamples = 0;
        imuError = "";
        imuRotation.clearAxis();
        try { imu = opMode == null ? null : opMode.hardwareMap.get(IMU.class, "imu"); }
        catch (RuntimeException ignored) { imu = null; }
    }

    @Override protected void showPrepare() {
        context.getTelemetry().addLine("Clear the floor around the robot.");
        if (context.getDrivetrain().isHolonomic()) {
            context.getTelemetry().addLine("It will drive forward, left, then turn.");
        } else {
            context.getTelemetry().addLine("It will drive forward, then turn.");
        }
        context.getTelemetry().addLine("Each move uses 50% power for 0.5 seconds.");
        context.getTelemetry().addLine("A: begin");
    }

    @Override protected void beginRecording() { beginMotion(); }

    @Override protected boolean record() {
        if (!context.getLocalizer().isLastMeasurementValid()) { invalidSamples++; }
        Motion motion = motions.get(motionIndex);
        updateImu(motion);
        context.getTelemetry().addData("Motion", display(motion));
        context.getTelemetry().addData("Stage", stage);

        if (stage == Stage.DRIVE) {
            command(motion);
            if (System.nanoTime() - stageStarted >= DRIVE_NANOS) {
                context.stop();
                stage = Stage.SETTLE;
                stageStarted = System.nanoTime();
            }
            return false;
        }

        context.stop();
        if (System.nanoTime() - stageStarted < SETTLE_NANOS) { return false; }
        CalibrationSnapshot end = context.getAdapter().snapshot(context.getLocalizer());
        responses.add(response(motion, start, end));
        if (motion == Motion.FORWARD) {
            trials.add(new DirectionTrial(CalibrationAxis.FORWARD, start, end));
        } else if (motion == Motion.STRAFE) {
            trials.add(new DirectionTrial(CalibrationAxis.STRAFE, start, end));
        }
        motionIndex++;
        if (motionIndex >= motions.size()) {
            candidate = context.getAdapter().fitDirections(context.getConfig(), trials);
            return true;
        }
        beginMotion();
        return false;
    }

    @Override protected void showReview() {
        for (Response response : responses) {
            context.getTelemetry().addData(response.label,
                    classify(response.primary, response.minimum, response.autoCorrect));
        }
        if (invalidSamples > 0) {
            context.getTelemetry().addData("Invalid samples", invalidSamples);
        }
        if (imu != null && imuRotation.hasSpinAxis()) {
            context.getTelemetry().addData("IMU spin axis (x, y, z)", String.format(
                    java.util.Locale.US, "%.3f, %.3f, %.3f",
                    imuRotation.getSpinAxisX(), imuRotation.getSpinAxisY(),
                    imuRotation.getSpinAxisZ()));
        } else if (!imuError.isEmpty()) {
            context.getTelemetry().addData("IMU check", imuError);
        }
    }

    @Override protected boolean accept() {
        return context.acceptGeometry("DIRECTIONS", candidate);
    }

    private void beginMotion() {
        context.stop();
        start = context.getAdapter().snapshot(context.getLocalizer());
        stage = Stage.DRIVE;
        stageStarted = System.nanoTime();
        if (motions.get(motionIndex) == Motion.TURN && imu != null) {
            try { imuRotation.resetMeasurement(readQuaternion()); }
            catch (RuntimeException failure) {
                imuError = "Could not read device 'imu': " + failure.getMessage();
                imu = null;
            }
        }
    }

    private void command(Motion motion) {
        if (motion == Motion.FORWARD) {
            context.getDrivetrain().moveWithVectors(POWER, 0.0, 0.0);
        } else if (motion == Motion.STRAFE) {
            context.getDrivetrain().moveWithVectors(0.0, POWER, 0.0);
        } else {
            context.getDrivetrain().moveWithVectors(0.0, 0.0, POWER);
        }
    }

    private void updateImu(Motion motion) {
        if (motion != Motion.TURN || imu == null) { return; }
        try { imuRotation.update(readQuaternion()); }
        catch (RuntimeException failure) {
            imuError = "Device 'imu' stopped returning orientation: " + failure.getMessage();
            imu = null;
        }
    }

    private Quaternion readQuaternion() { return imu.getRobotOrientationAsQuaternion(); }

    private static Response response(Motion motion, CalibrationSnapshot start,
                                     CalibrationSnapshot end) {
        Pose startPose = start.pose;
        Pose endPose = end.pose;
        Vector robotDelta = endPose.getVec().minus(startPose.getVec())
                .rotate(startPose.getHeading().times(-1.0));
        double heading = Angle.wrap(endPose.getHeading().getRad()
                - startPose.getHeading().getRad());
        boolean autoCorrect = start.rawTicks.length > 0 && end.rawTicks.length > 0;
        if (motion == Motion.FORWARD) {
            return new Response("Forward", robotDelta.getX().getIn(),
                    MIN_TRANSLATION_INCHES, autoCorrect);
        }
        if (motion == Motion.STRAFE) {
            return new Response("Strafe", robotDelta.getY().getIn(),
                    MIN_TRANSLATION_INCHES, autoCorrect);
        }
        return new Response("Turn", heading, MIN_ROTATION_RADIANS,
                false);
    }

    private static String classify(double value, double minimum, boolean autoCorrect) {
        if (value >= minimum) { return "CORRECT"; }
        if (value <= -minimum) { return autoCorrect ? "REVERSED (WILL FIX)" : "REVERSED"; }
        return "NO RESPONSE";
    }

    private static String display(Motion motion) {
        if (motion == Motion.STRAFE) { return "strafe left"; }
        if (motion == Motion.TURN) { return "turn counterclockwise"; }
        return "drive forward";
    }

    private static final class Response {
        final String label;
        final double primary;
        final double minimum;
        final boolean autoCorrect;

        Response(String label, double primary, double minimum, boolean autoCorrect) {
            this.label = label;
            this.primary = primary;
            this.minimum = minimum;
            this.autoCorrect = autoCorrect;
        }
    }
}
