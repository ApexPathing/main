package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Arrays;

import core.ApexStorage;
import core.Follower;
import feedforward.MotionParameters;
import geometry.GeometryFactory;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.movements.FollowerMovement;
import paths.movements.Path;

/**
 * Test autonomous OpMode for Apex Pathing that uses the {@link ExampleAutoPath}. Make sure the
 * robot has been tuned with the {@link FollowerTuner} before running this OpMode. This OpMode will
 * first follow the test path, then follow the test turn, and finally stop.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Autonomous(name = "Apex Auto Test", group = "Apex Pathing")
public class AutoTest extends LinearOpMode {
    private static final double STAGE_TIMEOUT_SECONDS = 50.0;
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double HEADING_TOLERANCE_DEGREES = 5.0;
    private static final double TELEMETRY_INTERVAL_SECONDS = 0.10;
    private static final double VELOCITY_LOG_INTERVAL_SECONDS = 0.02;
    private static final long CONTROL_LOOP_NANOS = 20_000_000L;
    // This is a coarse stress-test curve, not the endpoint acceptance tolerance. Leave enough
    // margin for the measured transient (~7.2 in in FTCodeSim) while still catching gross drift.
    private static final double CROSS_TRACK_TOLERANCE_INCHES = 15;

    ExampleAutoPath path;
    AutoState currentState = AutoState.OUTBOUND_CURVE;
    private final ElapsedTime stageTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private String failureReason = "None";
    private int passedStages;
    private double lastPositionError;
    private double lastHeadingError;
    private double maximumCrossTrackError;
    private double lastMaximumCrossTrackError;
    private FileWriter outboundVelocityCsv;
    private String outboundVelocityCsvPath = "Not started";
    private String outboundVelocityCsvError;
    private int outboundVelocityRowsSinceFlush;
    private double lastOutboundVelocityLogSeconds = Double.NEGATIVE_INFINITY;
    private static final String[] DEMAND_NAMES = {"cross-track", "tangent correction",
            "heading correction", "centripetal", "forward velocity", "heading velocity",
            "drive feedforward", "heading feedforward"};
    private final double[] peakDemand = new double[DEMAND_NAMES.length];
    private final int[] dominantSaturationFrames = new int[DEMAND_NAMES.length];
    private long demandSamples;
    private long saturatedDemandSamples;
    private double squaredTotalDemand;
    private double peakTotalDemand;
    private final double[] stagePeakDemand = new double[3];
    private final double[] stageSquaredDemand = new double[3];
    private final long[] stageSaturatedSamples = new long[3];
    private final double[] movementDurations = new double[5];
    private final double[] firstToleranceTimes = new double[5];

    enum AutoState {
        OUTBOUND_CURVE,
        POINT_TURN,
        REVERSE_RETURN,
        STRAFE_OUT,
        STRAFE_BACK,
        COMPLETE,
        FAILED
    }

    @Override
    public void runOpMode() {
        resetRunState();
        Follower follower = new Follower(new Constants(), hardwareMap);
        path = new ExampleAutoPath(follower, GeometryFactory.PoseMirror.NONE);

        telemetry.addLine("Apex follower self-test: curve, turn, reverse return, and strafe.");
        telemetry.addLine("Press Start to begin");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) { return; }

        follower.setPose(Pose.zero());
        openOutboundVelocityCsv();
        startStage(follower, currentState);

        while (opModeIsActive()) {
            long loopStartedNanos = System.nanoTime();
            follower.update();
            sampleCommandDemand(follower);
            logOutboundVelocitySample(follower);
            Pose pose = follower.getPose();
            recordFirstToleranceEntry(pose);
            maximumCrossTrackError = Math.max(maximumCrossTrackError,
                    Math.abs(follower.getCrossTrackErrorIn()));

            if (!isTerminal(currentState)) {
                if (stageTimer.seconds() > STAGE_TIMEOUT_SECONDS) {
                    fail(follower, "Stage exceeded " + STAGE_TIMEOUT_SECONDS + " seconds");
                } else if (!follower.isBusy()) {
                    finishStage(follower, pose);
                }
            }

            if (telemetryTimer.seconds() >= TELEMETRY_INTERVAL_SECONDS ||
                    isTerminal(currentState)) {
                if (currentState == AutoState.COMPLETE) {
                    telemetry.addLine("PASS: all Apex follower checks completed.");
                } else if (currentState == AutoState.FAILED) {
                    telemetry.addLine("FAIL: " + failureReason);
                }

                telemetry.addData("Current check", currentState);
                telemetry.addData("Passed checks", passedStages + " / 5");
                telemetry.addData("Stage time (s)", stageTimer.seconds());
                telemetry.addData("Follower busy", follower.isBusy());
                telemetry.addData("Callback state", path.callbackMessage);
                telemetry.addData("Last endpoint position error (in)", lastPositionError);
                telemetry.addData("Last endpoint heading error (deg)", lastHeadingError);
                telemetry.addData("Current maximum cross-track error (in)",
                        maximumCrossTrackError);
                telemetry.addData("Last maximum cross-track error (in)",
                        lastMaximumCrossTrackError);
                telemetry.addData("Outbound velocity CSV", outboundVelocityCsvPath);
                telemetry.addData("Command demand", getCommandDemandReport());
                if (outboundVelocityCsvError != null) {
                    telemetry.addData("Velocity CSV warning", outboundVelocityCsvError);
                }
                telemetry.addData("X (in)", pose.getX().getIn());
                telemetry.addData("Y (in)", pose.getY().getIn());
                telemetry.addData("Heading (deg)", pose.getHeading().getDeg());
                telemetry.update();
                telemetryTimer.reset();
            }
            waitForNextControlLoop(loopStartedNanos);
        }

        closeOutboundVelocityCsv();
        follower.stop();
    }

    private void finishStage(Follower follower, Pose actualPose) {
        movementDurations[currentState.ordinal()] = stageTimer.seconds();
        if (currentState == AutoState.OUTBOUND_CURVE) { closeOutboundVelocityCsv(); }
        FollowerMovement completed = movementFor(currentState);
        Pose expectedPose = completed.getEndPose();
        lastPositionError = actualPose.distanceTo(expectedPose).getIn();
        lastHeadingError = Math.abs(actualPose.getHeading()
                .getShortestAngleTo(expectedPose.getHeading()).getDeg());
        lastMaximumCrossTrackError = maximumCrossTrackError;

        if (!Double.isFinite(lastPositionError) || !Double.isFinite(lastHeadingError) ||
                !Double.isFinite(maximumCrossTrackError)) {
            fail(follower, "Non-finite localization/tracking result in " + currentState);
            return;
        }
        if (lastPositionError > POSITION_TOLERANCE_INCHES ||
                lastHeadingError > HEADING_TOLERANCE_DEGREES) {
            fail(follower, "Endpoint tolerance missed in " + currentState +
                    ": position=" + lastPositionError + " in, heading=" +
                    lastHeadingError + " deg");
            return;
        }
        if (currentState != AutoState.POINT_TURN &&
                maximumCrossTrackError > CROSS_TRACK_TOLERANCE_INCHES) {
            fail(follower, "Cross-track error exceeded " + CROSS_TRACK_TOLERANCE_INCHES +
                    " inches in " + currentState + ": " + maximumCrossTrackError + " in");
            return;
        }
        if (!requiredCallbackTriggered(currentState)) {
            fail(follower, "Expected callback did not run in " + currentState);
            return;
        }

        passedStages++;
        currentState = nextState(currentState);
        if (currentState == AutoState.COMPLETE) {
            follower.stop();
        } else {
            startStage(follower, currentState);
        }
    }

    private void startStage(Follower follower, AutoState state) {
        maximumCrossTrackError = 0.0;
        stageTimer.reset();
        follower.follow(movementFor(state));
    }

    private void fail(Follower follower, String reason) {
        closeOutboundVelocityCsv();
        follower.stop();
        failureReason = reason;
        currentState = AutoState.FAILED;
    }

    private FollowerMovement movementFor(AutoState state) {
        switch (state) {
            case OUTBOUND_CURVE: return path.testPath;
            case POINT_TURN: return path.testTurn;
            case REVERSE_RETURN: return path.returnPath;
            case STRAFE_OUT: return path.strafeOutPath;
            case STRAFE_BACK: return path.strafeBackPath;
            default: throw new IllegalArgumentException("No movement for terminal state " + state);
        }
    }

    private boolean requiredCallbackTriggered(AutoState state) {
        switch (state) {
            case OUTBOUND_CURVE: return path.outboundCallbackTriggered;
            case POINT_TURN: return path.turnCallbackTriggered;
            case REVERSE_RETURN: return path.returnCallbackTriggered;
            default: return true;
        }
    }

    static AutoState nextState(AutoState state) {
        switch (state) {
            case OUTBOUND_CURVE: return AutoState.POINT_TURN;
            case POINT_TURN: return AutoState.REVERSE_RETURN;
            case REVERSE_RETURN: return AutoState.STRAFE_OUT;
            case STRAFE_OUT: return AutoState.STRAFE_BACK;
            case STRAFE_BACK: return AutoState.COMPLETE;
            default: return state;
        }
    }

    private static boolean isTerminal(AutoState state) {
        return state == AutoState.COMPLETE || state == AutoState.FAILED;
    }

    /** Exposes the initial curve for simulation verification and diagnostics. */
    public Path getOutboundPath() { return path == null ? null : path.testPath; }

    public String getOutboundVelocityCsvPath() { return outboundVelocityCsvPath; }

    public String getOutboundVelocityCsvError() { return outboundVelocityCsvError; }

    /** Returns the elapsed motion time across all five completed checks. */
    public double getTotalMovementTimeSeconds() {
        double total = 0.0;
        for (double duration : movementDurations) { total += duration; }
        return total;
    }

    /** Yields to hardware while maintaining a complete 20 ms control-loop period. */
    private void waitForNextControlLoop(long loopStartedNanos) {
        long deadline = loopStartedNanos + CONTROL_LOOP_NANOS;
        if (deadline - System.nanoTime() > 6_000_000L) { sleep(5); }
        while (opModeIsActive() && System.nanoTime() < deadline) { Thread.yield(); }
    }

    /** Returns the average active follower update period in milliseconds. */
    public double getAverageLoopMilliseconds() {
        return demandSamples == 0 ? 0.0 :
                1000.0 * getTotalMovementTimeSeconds() / demandSamples;
    }

    /** Returns a concise timing breakdown for performance regression tests. */
    public String getMovementTimingReport() {
        return String.format(Locale.US,
                "outbound=%.3fs(+%.3fs settle), turn=%.3fs(+%.3fs settle), " +
                        "return=%.3fs(+%.3fs settle), strafeOut=%.3fs(+%.3fs settle), " +
                        "strafeBack=%.3fs(+%.3fs settle), total=%.3fs",
                movementDurations[0], settlingDelay(0),
                movementDurations[1], settlingDelay(1),
                movementDurations[2], settlingDelay(2),
                movementDurations[3], settlingDelay(3),
                movementDurations[4], settlingDelay(4),
                getTotalMovementTimeSeconds());
    }

    private double settlingDelay(int stage) {
        return Double.isFinite(firstToleranceTimes[stage])
                ? Math.max(0.0, movementDurations[stage] - firstToleranceTimes[stage])
                : movementDurations[stage];
    }

    private void recordFirstToleranceEntry(Pose actualPose) {
        int stage = currentState.ordinal();
        if (stage >= movementDurations.length || Double.isFinite(firstToleranceTimes[stage])) {
            return;
        }
        Pose expected = movementFor(currentState).getEndPose();
        double positionError = actualPose.distanceTo(expected).getIn();
        double headingError = Math.abs(actualPose.getHeading()
                .getShortestAngleTo(expected.getHeading()).getDeg());
        boolean inside = currentState == AutoState.POINT_TURN
                ? headingError <= 2.0
                : positionError <= 1.0 && headingError <= 2.0;
        if (inside) { firstToleranceTimes[stage] = stageTimer.seconds(); }
    }

    /** Summarizes raw controller demand before command normalization. */
    public String getCommandDemandReport() {
        double rms = demandSamples == 0 ? 0.0 :
                Math.sqrt(squaredTotalDemand / demandSamples);
        StringBuilder report = new StringBuilder(String.format(Locale.US,
                "samples=%d, saturated=%.1f%%, total peak=%.3f, total RMS=%.3f",
                demandSamples, demandSamples == 0 ? 0.0 :
                        100.0 * saturatedDemandSamples / demandSamples,
                peakTotalDemand, rms));
        for (int i = 0; i < DEMAND_NAMES.length; i++) {
            report.append(String.format(Locale.US, ", %s peak=%.3f, dominant=%d",
                    DEMAND_NAMES[i], peakDemand[i], dominantSaturationFrames[i]));
        }
        String[] stageNames = {"corrective", "velocity", "feedforward"};
        for (int i = 0; i < stageNames.length; i++) {
            double stageRms = demandSamples == 0 ? 0.0 :
                    Math.sqrt(stageSquaredDemand[i] / demandSamples);
            report.append(String.format(Locale.US,
                    ", %s stage peak=%.3f RMS=%.3f over-capacity=%.1f%%",
                    stageNames[i], stagePeakDemand[i], stageRms,
                    demandSamples == 0 ? 0.0 :
                            100.0 * stageSaturatedSamples[i] / demandSamples));
        }
        return report.toString();
    }

    private void sampleCommandDemand(Follower follower) {
        Follower.CommandDemand demand = follower.getLastCommandDemand();
        if (!demand.available) { return; }
        double[] components = {demand.crossTrack, demand.tangentCorrection,
                demand.headingCorrection, demand.centripetal, demand.forwardVelocity,
                demand.headingVelocity, demand.driveFeedforward, demand.headingFeedforward};
        double[] stages = {demand.correctiveTotal, demand.velocityTotal,
                demand.feedforwardTotal};
        demandSamples++;
        squaredTotalDemand += demand.total * demand.total;
        peakTotalDemand = Math.max(peakTotalDemand, demand.total);
        int dominant = 0;
        for (int i = 0; i < components.length; i++) {
            peakDemand[i] = Math.max(peakDemand[i], components[i]);
            if (components[i] > components[dominant]) { dominant = i; }
        }
        if (demand.total > 1.0) {
            saturatedDemandSamples++;
            dominantSaturationFrames[dominant]++;
        }
        for (int i = 0; i < stages.length; i++) {
            stagePeakDemand[i] = Math.max(stagePeakDemand[i], stages[i]);
            stageSquaredDemand[i] += stages[i] * stages[i];
            if (stages[i] > 1.0) { stageSaturatedSamples[i]++; }
        }
    }

    private void resetRunState() {
        closeOutboundVelocityCsv();
        currentState = AutoState.OUTBOUND_CURVE;
        failureReason = "None";
        passedStages = 0;
        lastPositionError = lastHeadingError = maximumCrossTrackError =
                lastMaximumCrossTrackError = 0.0;
        outboundVelocityCsvPath = "Not started";
        outboundVelocityCsvError = null;
        outboundVelocityRowsSinceFlush = 0;
        lastOutboundVelocityLogSeconds = Double.NEGATIVE_INFINITY;
        telemetryTimer.reset();
        demandSamples = saturatedDemandSamples = 0;
        squaredTotalDemand = peakTotalDemand = 0.0;
        Arrays.fill(peakDemand, 0.0);
        Arrays.fill(dominantSaturationFrames, 0);
        Arrays.fill(stagePeakDemand, 0.0);
        Arrays.fill(stageSquaredDemand, 0.0);
        Arrays.fill(stageSaturatedSamples, 0);
        Arrays.fill(movementDurations, 0.0);
        Arrays.fill(firstToleranceTimes, Double.NaN);
    }

    private void openOutboundVelocityCsv() {
        if (!path.testPath.isProfiled()) {
            outboundVelocityCsvPath = "Unavailable: testPath is not profiled";
            return;
        }
        try {
            File directory = ApexStorage.getDirectory();
            if (!directory.exists() && !directory.mkdirs()) {
                throw new IOException("Could not create " + directory.getAbsolutePath());
            }
            String timestamp = new SimpleDateFormat(
                    "yyyyMMdd_HHmmss_SSS", Locale.US).format(new Date());
            File file = new File(directory,
                    "auto-test-outbound-velocity_" + timestamp + ".csv");
            outboundVelocityCsv = new FileWriter(file);
            outboundVelocityCsvPath = file.getAbsolutePath();
            outboundVelocityCsv.write(
                    "elapsed_s,path_distance_in,target_velocity_in_s,raw_velocity_in_s," +
                            "kalman_velocity_in_s,target_accel_in_s2,target_omega_rad_s," +
                            "target_alpha_rad_s2,profile_power,cross_track_error_in," +
                            "curvature_in_inv,target_heading_rad,heading_error_rad," +
                            "cross_track_power,tangent_correction_power,heading_correction_power," +
                            "centripetal_power,forward_velocity_power,heading_velocity_power," +
                            "drive_feedforward_power,heading_feedforward_power,total_demand," +
                            "command_power,saturated\n");
        } catch (IOException e) {
            outboundVelocityCsv = null;
            outboundVelocityCsvPath = "Unavailable";
            outboundVelocityCsvError = e.getMessage();
        }
    }

    private void logOutboundVelocitySample(Follower follower) {
        if (outboundVelocityCsv == null || currentState != AutoState.OUTBOUND_CURVE) { return; }
        double elapsed = stageTimer.seconds();
        if (elapsed - lastOutboundVelocityLogSeconds < VELOCITY_LOG_INTERVAL_SECONDS) { return; }
        lastOutboundVelocityLogSeconds = elapsed;

        PathSegment segment = path.testPath.getParametricPath();
        double t = follower.getBestT();
        Vector closestPoint = segment.getPosition(t);
        double remaining = segment.getDistanceToEndIn(closestPoint, t);
        double traveled = segment.getLengthIn() - remaining;
        MotionParameters target = path.testPath.getFeedforwardLut().getFFParams(traveled);
        Vector tangent = segment.getFirstDerivative(t).normalize();
        double rawVelocity = follower.getRawVelocity().getVec().dot(tangent).getIn();
        double kalmanVelocity = follower.getVelocity().getVec().dot(tangent).getIn();
        Vector finalTangent = segment.getFirstDerivative(1.0).normalize();
        double targetHeading = path.testPath.getInterpolator()
                .getHeadingTarg(remaining, segment.getFirstDerivative(t), finalTangent).getRad();
        double headingError = follower.getPose().getHeading().getShortestAngleTo(
                geometry.Angle.fromRad(targetHeading)).getRad();
        double crossTrackError = follower.getCrossTrackErrorIn();
        double curvature = segment.getSignedCurvature(t);
        Follower.CommandDemand demand = follower.getLastCommandDemand();
        double commandPower = Math.max(Math.max(
                        Math.abs(follower.getDrivetrain().getLastFlPower()),
                        Math.abs(follower.getDrivetrain().getLastFrPower())),
                Math.max(Math.abs(follower.getDrivetrain().getLastBlPower()),
                        Math.abs(follower.getDrivetrain().getLastBrPower())));

        try {
            outboundVelocityCsv.write(String.format(
                    Locale.US,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f," +
                            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f," +
                            "%.6f,%.6f,%.6f,%.6f,%s%n",
                    elapsed, traveled, target.getTangentialVel(), rawVelocity,
                    kalmanVelocity, target.getTangentialAccel(), target.getAngularVel(),
                    target.getAngularAccel(), target.getMotorPower(), crossTrackError, curvature,
                    targetHeading, headingError, demand.crossTrack, demand.tangentCorrection,
                    demand.headingCorrection, demand.centripetal, demand.forwardVelocity,
                    demand.headingVelocity, demand.driveFeedforward, demand.headingFeedforward,
                    demand.total,
                    commandPower, commandPower >= 0.98));
            outboundVelocityRowsSinceFlush++;
            if (outboundVelocityRowsSinceFlush >= 25) {
                outboundVelocityCsv.flush();
                outboundVelocityRowsSinceFlush = 0;
            }
        } catch (IOException e) {
            outboundVelocityCsvError = e.getMessage();
            closeOutboundVelocityCsv();
        }
    }

    private void closeOutboundVelocityCsv() {
        if (outboundVelocityCsv == null) { return; }
        try {
            outboundVelocityCsv.close();
        } catch (IOException e) {
            outboundVelocityCsvError = e.getMessage();
        } finally {
            outboundVelocityCsv = null;
        }
    }
}
