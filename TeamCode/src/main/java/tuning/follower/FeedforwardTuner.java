package tuning.follower;

import tuning.TuningCsvWriter;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import geometry.AngleUnit;
import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import localizers.util.LowPassFilter;
import localizers.BaseLocalizer;

/**
 * Tunes angular and translational feedforward. Automatic tuning fits kS/kV from steady-speed
 * holding runs, then kA from accelerating integral windows. Independent feedforward-only runs
 * must pass in both directions before either axis's candidate is applied.
 *
 * @author Joel H - 7842a
 */
public class FeedforwardTuner extends TuningPhase {
    private static final double DYNAMIC_TIME = 1.25;
    private static final double INTEGRATION_WINDOW_SECONDS = 0.20;
    private static final double CHARACTERIZATION_POWER = 0.70;
    private static final double AUTO_SETTLE_TIME = 0.40;
    private static final double SIM_STAGING_OFFSET = 55.0;
    private static final double STATIONARY_LINEAR_SPEED_IN_PER_SEC = 1.0;
    private static final double STATIONARY_ANGULAR_SPEED_RAD_PER_SEC = 0.10;
    private static final double MANUAL_DRIVE_DISTANCE_INCHES = 48.0;
    private static final double MANUAL_TURN_ANGLE_RADIANS = Math.PI;
    private static final double MANUAL_SETTLE_TIMEOUT_SECONDS = 1.5;
    private static final double MANUAL_DRIVE_CRUISE_BOUND_IN_PER_SEC = 1.0;
    private static final double MANUAL_ANGULAR_CRUISE_BOUND_RAD_PER_SEC = Math.toRadians(1.0);

    private enum Coefficient { ANGULAR_KV, TRANSLATIONAL_KV, ANGULAR_KA, TRANSLATIONAL_KA }

    private enum ManualState { IDLE, DRIVE, SETTLING, ANGULAR }
    private ManualState manualState = ManualState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private TrapezoidProfile driveProfile;
    private TrapezoidProfile angularProfile;

    private Coefficient selected = Coefficient.ANGULAR_KV;

    private enum Axis { ANGULAR, TRANSLATIONAL }
    private enum Excitation { DYNAMIC, HOLD, VALIDATE }
    private enum AutoStage { PROMPT, RUNNING, SETTLING, FITTING, FAILED, DONE }

    private static final class AutoRun {
        final Axis axis;
        final Excitation excitation;
        final boolean forward;
        final double speedFraction;

        AutoRun(Axis axis, Excitation excitation, boolean forward) {
            this(axis, excitation, forward, 0.0);
        }

        AutoRun(Axis axis, Excitation excitation, boolean forward, double speedFraction) {
            this.axis = axis;
            this.excitation = excitation;
            this.forward = forward;
            this.speedFraction = speedFraction;
        }
    }

    private static final AutoRun[] AUTO_RUNS = {
            new AutoRun(Axis.ANGULAR, Excitation.DYNAMIC, true),
            new AutoRun(Axis.ANGULAR, Excitation.DYNAMIC, false),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.DYNAMIC, true),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.DYNAMIC, false),
            new AutoRun(Axis.ANGULAR, Excitation.HOLD, true, .15),
            new AutoRun(Axis.ANGULAR, Excitation.HOLD, false, .15),
            new AutoRun(Axis.ANGULAR, Excitation.HOLD, true, .35),
            new AutoRun(Axis.ANGULAR, Excitation.HOLD, false, .35),
            new AutoRun(Axis.ANGULAR, Excitation.HOLD, true, .55),
            new AutoRun(Axis.ANGULAR, Excitation.HOLD, false, .55),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.HOLD, true, .15),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.HOLD, false, .15),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.HOLD, true, .35),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.HOLD, false, .35),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.HOLD, true, .55),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.HOLD, false, .55),
            new AutoRun(Axis.ANGULAR, Excitation.VALIDATE, true),
            new AutoRun(Axis.ANGULAR, Excitation.VALIDATE, false),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.VALIDATE, true),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.VALIDATE, false)
    };
    private static final int VALIDATION_START = AUTO_RUNS.length - 4;
    private static final double HOLD_SECONDS = 1.8;

    static final class HoldingPoint {
        final double velocity, power;
        HoldingPoint(double velocity, double power) { this.velocity=velocity; this.power=power; }
    }

    /** Fit speed terms from independent holding runs, then acceleration from powered windows. */
    static double[] fitSeparated(List<HoldingPoint> holds, List<IntegralObservation> windows) {
        if (holds.size() < 6) { return new double[]{Double.NaN, Double.NaN, Double.NaN}; }
        double sx=0, sy=0, sxx=0, sxy=0;
        for (HoldingPoint p : holds) { sx+=p.velocity; sy+=p.power; sxx+=p.velocity*p.velocity; sxy+=p.velocity*p.power; }
        double n=holds.size(), determinant=n*sxx-sx*sx;
        if (determinant < 1e-9) { return new double[]{Double.NaN, Double.NaN, Double.NaN}; }
        double kv=(n*sxy-sx*sy)/determinant, ks=(sy-kv*sx)/n;
        // An affine fit to curved holding-power data can systematically overdrive the
        // middle speeds. Keep the fitted slope, but do not exceed any measured hold.
        // Velocity feedback can supply the small remaining load on the real path.
        for (HoldingPoint p : holds) { ks = Math.min(ks, p.power - kv * p.velocity); }
        double numerator=0, denominator=0;
        int accelerationWindows = 0;
        for (IntegralObservation w : windows) {
            if (w.velocityChange <= 0 || w.signTime <= 0) { continue; }
            numerator+=w.velocityChange*(w.powerTime-ks*w.signTime-kv*w.distance);
            denominator+=w.velocityChange*w.velocityChange;
            accelerationWindows++;
        }
        return new double[]{ks, kv, accelerationWindows >= 4 && denominator>1e-9
                ? numerator/denominator : Double.NaN};
    }

    static boolean physicalFit(double[] fit) {
        return Double.isFinite(fit[0]) && fit[0]>=0 && fit[0]<1 &&
                Double.isFinite(fit[1]) && fit[1]>0 && Double.isFinite(fit[2]) && fit[2]>0;
    }

    /** One derivative-free integral-model observation. */
    static final class IntegralObservation {
        final double signTime, distance, velocityChange, powerTime;

        IntegralObservation(double signTime, double distance, double velocityChange,
                            double powerTime) {
            this.signTime = signTime;
            this.distance = distance;
            this.velocityChange = velocityChange;
            this.powerTime = powerTime;
        }
    }

    private final List<IntegralObservation> angularIntegralObservations = new ArrayList<>();
    private final List<IntegralObservation> translationalIntegralObservations = new ArrayList<>();
    private double windowSignTime, windowDistance, windowVelocityChange, windowPowerTime;
    private double windowStartVelocity, previousWindowVelocity, previousIntegrationTime;
    private boolean integrationWindowStarted;
    private AutoStage autoStage = AutoStage.PROMPT;
    private int autoRunIndex = 0;
    private String validationMessage = "Not run";
    private String csvPath = "Not written";
    private String csvError;
    private int manualRunNumber;
    private final ManualErrorMetrics manualDriveMetrics = new ManualErrorMetrics();
    private final ManualErrorMetrics manualAngularMetrics = new ManualErrorMetrics();
    private int manualSaturatedSamples;
    private double manualPeakError;
    private double manualPeakVelocity;
    private boolean manualSummaryWritten;
    private TuningCsvWriter manualCsv;
    private String manualCsvPath = "Not started";
    private final LowPassFilter commandPowerFilter = new LowPassFilter();
    private double lastAppliedCharacterizationPower;
    private final List<HoldingPoint> angularHolds = new ArrayList<>();
    private final List<HoldingPoint> translationHolds = new ArrayList<>();
    private double[] angularCandidate, translationCandidate;
    private double holdIntegral, holdLastTime, holdVelocitySum, holdPowerSum;
    private int holdSamples;
    private final double[] validationSquared = new double[4];
    private final int[] validationSamples = new int[4];
    private final boolean[] validationSaturated = new boolean[4];
    private final double[][] validationBias = new double[4][2];
    private final double[][] validationPhaseSquared = new double[4][2];
    private final int[][] validationPhaseSamples = new int[4][2];
    private TuningCsvWriter validationCsv;
    private int validationRefinements;

    public FeedforwardTuner(TunerContext context) {
        super(context);
    }

    @Override
    protected String getPhaseName() {
        return "Feedforward Refinement";
    }

    @Override
    protected boolean manualTuneIsPossible() {
        return true;
    }

    @Override
    protected boolean autoTuneIsPossible() {
        return true;
    }

    @Override
    protected void showPreRunInstructions() {
        context.getTelemetry().addLine(
                "Manual X tests drive forward 48 inches, settle, then turn around 180 degrees.");
        context.getTelemetry().addLine(
                "Leave a clear 48-inch lane and enough room for an in-place turn.");
    }

    @Override
    protected void init() {
        // Target half of the physical limits to avoid motor saturation during tuning.
        angularProfile = new TrapezoidProfile(
                context.constants.angularVelLimitRad / 2,
                context.constants.angularAccelLimitRad / 2,
                MANUAL_TURN_ANGLE_RADIANS
        );
        driveProfile = new TrapezoidProfile(
                context.constants.forwardVelLimitIn / 2,
                context.constants.forwardAccelLimitIn / 2,
                MANUAL_DRIVE_DISTANCE_INCHES
        );

        timer.reset();
        if (manualMode) {
            manualState = ManualState.IDLE;
            context.getFollower().stop();
            manualRunNumber = 0;
            manualDriveMetrics.reset();
            manualAngularMetrics.reset();
            manualCsv = TuningCsvWriter.open("manual_feedforward_response",
                    "record_type", "run", "time_s", "axis", "direction", "target_velocity",
                    "actual_velocity", "error", "raw_power", "applied_power", "saturated",
                    "included_in_kv_cruise_rms", "kv_cruise_rms", "ka_whole_run_rms");
            manualCsvPath = manualCsv.getPath();
        } else {
            restartAutomaticCharacterization();
        }
    }

    private void restartAutomaticCharacterization() {
        context.getFollower().stop();
        angularIntegralObservations.clear();
        translationalIntegralObservations.clear();
        angularHolds.clear();
        translationHolds.clear();
        angularCandidate=null;
        translationCandidate=null;
        validationRefinements = 0;
        Arrays.fill(validationSquared, 0);
        Arrays.fill(validationSamples, 0);
        Arrays.fill(validationSaturated, false);
        for (int i = 0; i < 4; i++) {
            Arrays.fill(validationBias[i], 0);
            Arrays.fill(validationPhaseSquared[i], 0);
            Arrays.fill(validationPhaseSamples[i], 0);
        }
        if (validationCsv != null) { validationCsv.close(); }
        validationCsv=TuningCsvWriter.open("feedforward_validation", "run", "axis", "direction",
                "time_s", "target_velocity", "measured_velocity", "power", "sample_used");
        validationMessage = "Collecting characterization data";
        csvPath = "Pending";
        csvError = null;
        autoRunIndex = 0;
        autoStage = AutoStage.PROMPT;
        if (usesMovingAverageVelocity()) {
            commandPowerFilter.setSampleSize(
                    context.getFollower().getLocalizer().getFilterWindowSize());
        } else {
            commandPowerFilter.setSampleSize(1);
        }
        lastAppliedCharacterizationPower = 0.0;
        timer.reset();
    }


    /**
     * Generates a 1-second cruise trapezoidal motion profile to evaluate feedforward consistency.
     */
    public static class TrapezoidProfile {
        private final double accel;
        private final double vel;
        private final double tAccelEnd;
        private final double tCruiseEnd;
        private final double tEnd;
        private final double totalDistance;

        public TrapezoidProfile(double vel, double accel) {
            this(vel, accel, Math.abs(vel) * (Math.abs(vel) / Math.abs(accel) + 1.0));
        }

        /** Builds an exact-distance trapezoid, falling back to a triangle when required. */
        public TrapezoidProfile(double vel, double accel, double distance) {
            this.vel = Math.abs(vel);
            this.accel = Math.abs(accel);
            this.totalDistance = Math.abs(distance);

            if (!Double.isFinite(this.vel) || !Double.isFinite(this.accel) ||
                    !Double.isFinite(this.totalDistance) || this.vel <= 0.0 ||
                    this.accel <= 0.0 || this.totalDistance <= 0.0) {
                throw new IllegalArgumentException(
                        "Trapezoid profile inputs must be finite and positive."
                );
            }

            this.tAccelEnd = Math.min(this.vel / this.accel,
                    Math.sqrt(this.totalDistance / this.accel));
            double peakVelocity = this.accel * this.tAccelEnd;
            double cruiseDistance = Math.max(0.0,
                    this.totalDistance - this.accel * this.tAccelEnd * this.tAccelEnd);
            this.tCruiseEnd = this.tAccelEnd + cruiseDistance / peakVelocity;
            this.tEnd = this.tCruiseEnd + this.tAccelEnd;
        }

        public double getAccel(double t) {
            if (t < 0 || t >= tEnd) {
                return 0.0;
            }
            if (t < tAccelEnd) {
                return accel;
            }
            if (t < tCruiseEnd) {
                return 0.0;
            }
            return -accel;
        }

        public double getVel(double t) {
            if (t < 0 || t >= tEnd) {
                return 0.0;
            }
            if (t < tAccelEnd) {
                return accel * t;
            }
            if (t < tCruiseEnd) {
                return accel * tAccelEnd;
            }
            // Deceleration: Max velocity minus what has been lost over time
            return accel * tAccelEnd - accel * (t - tCruiseEnd);
        }

        public double getTotalTime() {
            return tEnd;
        }

        public double getAccelEnd() {
            return tAccelEnd;
        }

        public double getCruiseEnd() {
            return tCruiseEnd;
        }

        public double getTotalDistance() {
            return totalDistance;
        }
    }

    @Override
    protected boolean manualTuned() {
        if (opMode.gamepad1.leftBumperWasPressed()) {
            selected = selected == Coefficient.ANGULAR_KV ? Coefficient.TRANSLATIONAL_KA :
                    Coefficient.values()[selected.ordinal() - 1];
        }
        if (opMode.gamepad1.rightBumperWasPressed()) {
            selected = selected == Coefficient.TRANSLATIONAL_KA ? Coefficient.ANGULAR_KV :
                    Coefficient.values()[selected.ordinal() + 1];
        }

        double change = manualChange();
        boolean coefficientsChanged = change != 0.0;
        if (change != 0.0) {
            if (selected == Coefficient.ANGULAR_KV) {
                context.constants.angularKV = Math.max(
                        0.0, context.constants.angularKV + change
                );
            } else if (selected == Coefficient.ANGULAR_KA) {
                context.constants.angularKA = Math.max(
                        0.0, context.constants.angularKA + change
                );
            } else if (selected == Coefficient.TRANSLATIONAL_KV) {
                context.constants.translationalKV = Math.max(
                        0.0, context.constants.translationalKV + change
                );
            } else if (selected == Coefficient.TRANSLATIONAL_KA) {
                context.constants.translationalKA = Math.max(
                        0.0, context.constants.translationalKA + change
                );
            }
            context.getFollower().setFeedforwardGains(
                    context.constants.translationalKV,
                    context.constants.translationalKA,
                    context.constants.angularKV,
                    context.constants.angularKA);
        }

        if (coefficientsChanged && manualState != ManualState.IDLE) {
            context.getFollower().stop();
            manualState = ManualState.IDLE;
        }

        if (opMode.gamepad1.xWasPressed()) {
            context.getFollower().stop();
            resetManualMetrics();
            manualState = ManualState.DRIVE;
            timer.reset();
        }

        Pose velocity = context.getFollower().getVelocity();
        double angularVel = velocity.getHeading(AngleUnit.RAD);
        double driveVel = velocity.getX().getIn();
        double time_sec = timer.seconds();
        double targetVel = 0.0;
        double currentVel = 0.0;

        switch (manualState) {
            case IDLE:
                context.getFollower().stop();
                break;
            case DRIVE:
                if (time_sec >= driveProfile.getTotalTime()) {
                    context.getFollower().stop();
                    manualState = ManualState.SETTLING;
                    timer.reset();
                    break;
                }
                targetVel = driveProfile.getVel(time_sec);
                currentVel = driveVel;
                double drivePow =
                        context.constants.translationalKV * targetVel +
                                context.constants.translationalKA * driveProfile.getAccel(time_sec) +
                                context.constants.translationalFeedforwardKS;
                double rawDrivePower = drivePow;
                double appliedDrivePower = clipManualPower(rawDrivePower);
                context.getFollower().getDrivetrain().moveWithVectors(
                        appliedDrivePower, 0.0, 0.0
                );
                recordManualFeedforwardSample(targetVel, currentVel,
                        driveProfile.getVel(driveProfile.getAccelEnd()),
                        time_sec < driveProfile.getCruiseEnd(),
                        MANUAL_DRIVE_CRUISE_BOUND_IN_PER_SEC,
                        rawDrivePower, appliedDrivePower);
                break;
            case SETTLING:
                context.getFollower().stop();
                boolean stationary = Math.hypot(
                        velocity.getX().getIn(), velocity.getY().getIn()) <=
                        STATIONARY_LINEAR_SPEED_IN_PER_SEC &&
                        Math.abs(angularVel) <= STATIONARY_ANGULAR_SPEED_RAD_PER_SEC;
                if ((time_sec >= AUTO_SETTLE_TIME && stationary) ||
                        time_sec >= MANUAL_SETTLE_TIMEOUT_SECONDS) {
                    manualState = ManualState.ANGULAR;
                    timer.reset();
                }
                break;
            case ANGULAR:
                if (time_sec >= angularProfile.getTotalTime()) {
                    context.getFollower().stop();
                    writeManualSummary();
                    manualState = ManualState.IDLE;
                    break;
                }
                targetVel = angularProfile.getVel(time_sec);
                currentVel = angularVel;
                double angularPow = manualAngularPower(
                        context.constants.angularKV, context.constants.angularKA,
                        context.constants.angularFeedforwardKS,
                        targetVel, angularProfile.getAccel(time_sec), 1.0);
                double appliedAngularPower = clipManualPower(angularPow);
                context.getFollower().getDrivetrain().moveWithVectors(
                        0.0, 0.0, appliedAngularPower);
                recordManualFeedforwardSample(targetVel, currentVel,
                        angularProfile.getVel(angularProfile.getAccelEnd()),
                        time_sec < angularProfile.getCruiseEnd(),
                        MANUAL_ANGULAR_CRUISE_BOUND_RAD_PER_SEC,
                        angularPow, appliedAngularPower);
                break;
        }

        addTunableValue("Angular KV", context.constants.angularKV,
                selected == Coefficient.ANGULAR_KV);
        addTunableValue("Angular KA", context.constants.angularKA,
                selected == Coefficient.ANGULAR_KA);
        addTunableValue("Translational KV", context.constants.translationalKV,
                selected == Coefficient.TRANSLATIONAL_KV);
        addTunableValue("Translational KA", context.constants.translationalKA,
                selected == Coefficient.TRANSLATIONAL_KA);
        context.getTelemetry().addData("Increment", number(increment));
        context.getTelemetry().addLine("Target Vel: " + number(targetVel));
        context.getTelemetry().addLine("Current Vel: " + number(currentVel) +
                (manualState == ManualState.ANGULAR ? " rad/s" :
                        manualState == ManualState.DRIVE ? " in/s" : ""));
        context.getTelemetry().addData("Velocity error", number(targetVel - currentVel));
        context.getTelemetry().addData("Drive kV cruise RMS", number(manualDriveMetrics.cruiseRms()));
        context.getTelemetry().addData("Angular kV cruise RMS", number(manualAngularMetrics.cruiseRms()));
        context.getTelemetry().addData("Drive kA whole-run RMS", number(manualDriveMetrics.wholeRunRms()));
        context.getTelemetry().addData("Angular kA whole-run RMS", number(manualAngularMetrics.wholeRunRms()));
        if (context.isDebugMode()) {
            context.getTelemetry().addData("Test state", manualState);
            context.getTelemetry().addData("Peak velocity error", manualPeakError);
            context.getTelemetry().addData("Peak measured velocity", manualPeakVelocity);
            int totalSamples = manualDriveMetrics.wholeRunSamples +
                    manualAngularMetrics.wholeRunSamples;
            context.getTelemetry().addData("Saturation", totalSamples == 0 ? "0.0%" :
                    Math.round(1000.0 * manualSaturatedSamples / totalSamples) / 10.0 + "%");
            context.getTelemetry().addData("Response CSV", manualCsvPath);
        }
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("LB/RB: select Value to tune");
        context.getTelemetry().addLine("X: Run/restart 48-inch drive + 180-degree turn test");
        context.getTelemetry().addLine("A: Save");
        context.getTelemetry().update();
        if (opMode.gamepad1.aWasPressed()) {
            context.getFollower().stop();
            writeManualSummary();
            if (manualCsv != null) { manualCsv.close(); }
            return true;
        }
        return false;
    }

    private void resetManualMetrics() {
        manualRunNumber++;
        manualDriveMetrics.reset();
        manualAngularMetrics.reset();
        manualSaturatedSamples = 0;
        manualPeakError = 0.0;
        manualPeakVelocity = 0.0;
        manualSummaryWritten = false;
    }

    private void recordManualFeedforwardSample(double targetVelocity, double actualVelocity,
                                               double cruiseVelocity, boolean beforeDeceleration,
                                               double cruiseEntryTolerance,
                                               double rawPower, double appliedPower) {
        if (!Double.isFinite(targetVelocity) || !Double.isFinite(actualVelocity)) { return; }
        double error = targetVelocity - actualVelocity;
        ManualErrorMetrics metrics;
        if (manualState == ManualState.DRIVE) {
            metrics = manualDriveMetrics;
        } else if (manualState == ManualState.ANGULAR) {
            metrics = manualAngularMetrics;
        } else {
            return;
        }
        boolean cruiseSample = metrics.record(targetVelocity, actualVelocity, cruiseVelocity,
                beforeDeceleration, cruiseEntryTolerance);
        manualPeakError = Math.max(manualPeakError, Math.abs(error));
        manualPeakVelocity = Math.max(manualPeakVelocity, Math.abs(actualVelocity));
        boolean saturated = Math.abs(rawPower) >= 1.0;
        if (saturated) { manualSaturatedSamples++; }
        if (manualCsv != null) {
            String direction = manualState == ManualState.ANGULAR
                    ? "COUNTERCLOCKWISE" : "FORWARD";
            manualCsv.writeRow("SAMPLE", manualRunNumber, timer.seconds(), manualState, direction,
                    targetVelocity, actualVelocity, error, rawPower, appliedPower, saturated,
                    cruiseSample, metrics.cruiseRms(), metrics.wholeRunRms());
        }
    }

    private void writeManualSummary() {
        if (manualCsv == null || manualSummaryWritten || manualRunNumber == 0) { return; }
        manualCsv.writeRow("SUMMARY", manualRunNumber, Double.NaN, "DRIVE_KV", "FORWARD",
                Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, false,
                false, manualDriveMetrics.cruiseRms(), Double.NaN);
        manualCsv.writeRow("SUMMARY", manualRunNumber, Double.NaN, "DRIVE_KA", "FORWARD",
                Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, false,
                false, Double.NaN, manualDriveMetrics.wholeRunRms());
        manualCsv.writeRow("SUMMARY", manualRunNumber, Double.NaN, "ANGULAR_KV",
                "COUNTERCLOCKWISE", Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, false, false, manualAngularMetrics.cruiseRms(), Double.NaN);
        manualCsv.writeRow("SUMMARY", manualRunNumber, Double.NaN, "ANGULAR_KA",
                "COUNTERCLOCKWISE", Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, false, false, Double.NaN, manualAngularMetrics.wholeRunRms());
        manualSummaryWritten = true;
    }

    /** Separates kV's reached-cruise evidence from kA's whole-profile evidence. */
    static final class ManualErrorMetrics {
        private boolean cruiseReached;
        private int cruiseSamples;
        private int wholeRunSamples;
        private double cruiseErrorSquared;
        private double wholeRunErrorSquared;

        boolean record(double targetVelocity, double actualVelocity, double cruiseVelocity,
                       boolean beforeDeceleration, double cruiseEntryTolerance) {
            double runError = targetVelocity - actualVelocity;
            wholeRunErrorSquared += runError * runError;
            wholeRunSamples++;
            if (!beforeDeceleration) { return false; }
            if (!cruiseReached &&
                    Math.abs(actualVelocity - cruiseVelocity) <= cruiseEntryTolerance) {
                cruiseReached = true;
            }
            if (!cruiseReached) { return false; }
            double cruiseError = cruiseVelocity - actualVelocity;
            cruiseErrorSquared += cruiseError * cruiseError;
            cruiseSamples++;
            return true;
        }

        double cruiseRms() {
            return cruiseSamples == 0 ? Double.NaN :
                    Math.sqrt(cruiseErrorSquared / cruiseSamples);
        }

        double wholeRunRms() {
            return wholeRunSamples == 0 ? Double.NaN :
                    Math.sqrt(wholeRunErrorSquared / wholeRunSamples);
        }

        void reset() {
            cruiseReached = false;
            cruiseSamples = 0;
            wholeRunSamples = 0;
            cruiseErrorSquared = 0.0;
            wholeRunErrorSquared = 0.0;
        }
    }

    static double manualAngularPower(double kV, double kA, double kS,
                                     double profileVelocity, double profileAcceleration,
                                     double direction) {
        double signedDirection = direction >= 0.0 ? 1.0 : -1.0;
        return kV * profileVelocity * signedDirection +
                kA * profileAcceleration * signedDirection + kS * signedDirection;
    }

    static double clipManualPower(double rawPower) {
        return Range.clip(rawPower, -1.0, 1.0);
    }

    @Override
    protected boolean routineMotionActive() {
        if (manualMode) { return manualState != ManualState.IDLE; }
        return autoStage == AutoStage.RUNNING || autoStage == AutoStage.SETTLING;
    }

    @Override
    protected boolean autoTuned() {
        if (autoStage == AutoStage.DONE) { return true; }

        if (autoStage == AutoStage.FAILED) {
            context.getFollower().stop();
            context.getTelemetry().addLine("Feedforward characterization did not pass validation.");
            context.getTelemetry().addData("Validation", validationMessage);
            context.getTelemetry().addLine("The previous feedforward values are still active.");
            context.getTelemetry().addLine("Press A to retry automatically.");
            context.getTelemetry().addLine("Press B to switch to manual tuning.");
            reportFitDiagnostics();
            context.getTelemetry().update();
            if (opMode.gamepad1.aWasPressed()) {
                restartAutomaticCharacterization();
            } else if (opMode.gamepad1.bWasPressed()) {
                manualMode = true;
                init();
            }
            return false;
        }

        if (autoStage == AutoStage.FITTING) {
            boolean validationComplete = autoRunIndex >= AUTO_RUNS.length;
            if (!validationComplete) {
                angularCandidate = fitSeparated(angularHolds, angularIntegralObservations);
                translationCandidate = fitSeparated(translationHolds, translationalIntegralObservations);
            }
            boolean angularValid = physicalFit(angularCandidate);
            boolean translationalValid = physicalFit(translationCandidate);
            if (angularValid && translationalValid && !validationComplete) {
                validationMessage = "Holding and acceleration fits ready; validating without feedback";
                autoStage = AutoStage.PROMPT;
                return false;
            }
            if (validationComplete) {
                angularValid &= validationPassed(0, Math.max(.20, .05 * context.constants.angularVelLimitRad))
                        && validationPassed(1, Math.max(.20, .05 * context.constants.angularVelLimitRad));
                translationalValid &= validationPassed(2, Math.max(2.0, .05 * context.constants.forwardVelLimitIn))
                        && validationPassed(3, Math.max(2.0, .05 * context.constants.forwardVelLimitIn));
            }
            if (validationComplete && (!angularValid || !translationalValid)
                    && validationRefinements < 2
                    && physicalFit(angularCandidate) && physicalFit(translationCandidate)) {
                // A small affine-model holding bias is observable without velocity feedback.
                // Refine only the failed axis, then require entirely fresh validation runs.
                if (!angularValid) { refineHoldingBias(angularCandidate, 0); }
                if (!translationalValid) { refineHoldingBias(translationCandidate, 2); }
                validationRefinements++;
                Arrays.fill(validationSquared, 0);
                Arrays.fill(validationSamples, 0);
                Arrays.fill(validationSaturated, false);
                for (int i = 0; i < 4; i++) {
                    Arrays.fill(validationBias[i], 0);
                    Arrays.fill(validationPhaseSquared[i], 0);
                    Arrays.fill(validationPhaseSamples[i], 0);
                }
                autoRunIndex = VALIDATION_START;
                autoStage = AutoStage.PROMPT;
                validationMessage = "Refining holding bias; repeating feedforward-only validation";
                return false;
            }
            if (angularValid && translationalValid) {
                context.constants.angularFeedforwardKS = angularCandidate[0];
                context.constants.angularKV = angularCandidate[1];
                context.constants.angularKA = angularCandidate[2];
                context.constants.translationalFeedforwardKS = translationCandidate[0];
                context.constants.translationalKV = translationCandidate[1];
                context.constants.translationalKA = translationCandidate[2];
                context.getFollower().setFeedforwardGains(
                        translationCandidate[0], translationCandidate[1], translationCandidate[2],
                        angularCandidate[0], angularCandidate[1], angularCandidate[2]);
            }
            validationMessage = "Angular " + (angularValid ? "PASSED" : "FAILED") +
                    "; Translation " + (translationalValid ? "PASSED" : "FAILED");
            if (!angularValid || !translationalValid) {
                validationMessage += "; no candidate values applied";
            }
            writeFitEvidenceCsv();
            if (validationCsv != null) { validationCsv.close(); }
            autoStage = angularValid && translationalValid ? AutoStage.DONE : AutoStage.FAILED;
            return autoStage == AutoStage.DONE;
        }

        AutoRun run = AUTO_RUNS[autoRunIndex];
        if (autoStage == AutoStage.PROMPT) {
            context.getFollower().stop();
            Pose velocity = context.getFollower().getVelocity();
            boolean stationary = Math.hypot(
                    velocity.getX().getIn(), velocity.getY().getIn()) <=
                    STATIONARY_LINEAR_SPEED_IN_PER_SEC &&
                    Math.abs(velocity.getHeading(AngleUnit.RAD)) <=
                            STATIONARY_ANGULAR_SPEED_RAD_PER_SEC;
            String direction = run.forward
                    ? (run.axis == Axis.ANGULAR ? "counterclockwise" : "forward")
                    : (run.axis == Axis.ANGULAR ? "clockwise" : "backward");
            context.getTelemetry().addLine("Feedforward characterization " +
                    (autoRunIndex + 1) + " / " + AUTO_RUNS.length);
            context.getTelemetry().addData("Test", run.axis + " " + run.excitation);
            context.getTelemetry().addData("Direction", direction);
            context.getTelemetry().addLine(run.axis == Axis.ANGULAR
                    ? "Place the robot where it can rotate safely."
                    : "Point the " + (run.forward ? "front" : "back") +
                            " toward at least 72 inches of clear space.");
            context.getTelemetry().addLine(
                    "Press A when the robot is stationary and the direction is safe.");
            context.getTelemetry().addLine(stationary
                    ? "Robot is stationary and ready."
                    : "Waiting for the robot to stop before starting.");
            context.getTelemetry().update();
            if (opMode.gamepad1.aWasPressed() && stationary) {
                double stagingX = run.axis == Axis.ANGULAR ? 0.0 :
                        (run.forward ? -SIM_STAGING_OFFSET : SIM_STAGING_OFFSET);
                positionRobotForSimulation(new Pose(
                        Vector.of(stagingX, 0.0, DistUnit.IN), Angle.fromRad(0.0)));
                commandPowerFilter.reset();
                lastAppliedCharacterizationPower = 0.0;
                holdIntegral=0;
                holdLastTime=0;
                holdVelocitySum=0;
                holdPowerSum=0;
                holdSamples=0;
                resetIntegrationWindow();
                timer.reset();
                autoStage = AutoStage.RUNNING;
            }
            return false;
        }

        double elapsed = timer.seconds();
        if (autoStage == AutoStage.RUNNING &&
                (run.excitation == Excitation.HOLD || run.excitation == Excitation.VALIDATE)) {
            runHoldingOrValidation(run, elapsed);
            context.getTelemetry().addData("Test", run.axis + " " + run.excitation);
            context.getTelemetry().addData("Progress", (autoRunIndex + 1) + " / " + AUTO_RUNS.length);
            context.getTelemetry().update();
            return false;
        }
        if (autoStage == AutoStage.RUNNING) {
            double duration = DYNAMIC_TIME;
            if (elapsed >= duration) {
                resetIntegrationWindow();
                timer.reset();
                context.getFollower().stop();
                lastAppliedCharacterizationPower = 0.0;
                autoStage = AutoStage.SETTLING;
            } else {
                // Preserve the legacy power-window alignment only when the compatibility moving
                // average is selected. Kalman state estimates are current-time estimates and must
                // not be shifted by the old seven-sample window.
                double alignedPower = commandPowerFilter.update(
                        lastAppliedCharacterizationPower).value();
                double power = CHARACTERIZATION_POWER;
                double signedPower = run.forward ? power : -power;
                if (run.axis == Axis.ANGULAR) {
                    context.getFollower().getDrivetrain().moveWithVectors(
                            0.0, 0.0, signedPower);
                } else {
                    context.getFollower().getDrivetrain().moveWithVectors(
                            signedPower, 0.0, 0.0);
                }
                lastAppliedCharacterizationPower = power;

                Pose velocity = context.getFollower().getVelocity();
                double rawVelocity = run.axis == Axis.ANGULAR
                        ? velocity.getHeading(AngleUnit.RAD)
                        : velocity.getX().getIn();
                recordIntegralSample(run, alignedPower, rawVelocity, elapsed);
            }
        } else if (autoStage == AutoStage.SETTLING) {
            context.getFollower().stop();
            if (elapsed >= AUTO_SETTLE_TIME) {
                autoRunIndex++;
                autoStage = autoRunIndex == VALIDATION_START || autoRunIndex >= AUTO_RUNS.length
                        ? AutoStage.FITTING : AutoStage.PROMPT;
                timer.reset();
            }
        }

        context.getTelemetry().addLine(feedforwardActionDescription(run));
        if (context.isDebugMode()) {
            context.getTelemetry().addData("Characterization", (autoRunIndex + 1) +
                    " / " + AUTO_RUNS.length);
            context.getTelemetry().addData("Test", run.axis + " " + run.excitation);
            context.getTelemetry().addData("Direction", run.forward ? "FORWARD / CCW" : "BACKWARD / CW");
            context.getTelemetry().addData("Angular acceleration windows", angularIntegralObservations.size());
            context.getTelemetry().addData("Translation acceleration windows", translationalIntegralObservations.size());
            context.getTelemetry().addData("CSV", csvPath);
        }
        context.getTelemetry().update();
        return false;
    }

    private boolean validationPassed(int run, double tolerance) {
        if (validationSaturated[run]) { return false; }
        for (int phase = 0; phase < 2; phase++) {
            if (!validationPhasePassed(validationPhaseSamples[run][phase],
                    validationPhaseSquared[run][phase], validationBias[run][phase], tolerance)) {
                return false;
            }
        }
        return true;
    }

    private void refineHoldingBias(double[] candidate, int firstRun) {
        int samples = validationPhaseSamples[firstRun][1] + validationPhaseSamples[firstRun + 1][1];
        if (samples < 20 || validationSaturated[firstRun] || validationSaturated[firstRun + 1]) { return; }
        double bias = (validationBias[firstRun][1] + validationBias[firstRun + 1][1]) / samples;
        candidate[0] = correctedHoldingKS(candidate[0], candidate[1], bias);
    }

    static double correctedHoldingKS(double ks, double kv, double speedBias) {
        return Math.max(0, Math.min(.99, ks - kv * speedBias));
    }

    static boolean validationPhasePassed(int samples, double squaredError,
                                         double signedError, double tolerance) {
        return samples >= 10 && Double.isFinite(squaredError) && Double.isFinite(signedError)
                && Math.sqrt(squaredError / samples) <= tolerance
                && Math.abs(signedError / samples) <= tolerance * .5;
    }

    /** Holding uses a temporary PI servo for identification; validation is feedforward only. */
    private void runHoldingOrValidation(AutoRun run, double elapsed) {
        boolean angular = run.axis == Axis.ANGULAR;
        boolean validating = run.excitation == Excitation.VALIDATE;
        double limit = angular ? context.constants.angularVelLimitRad : context.constants.forwardVelLimitIn;
        double duration = validating ? 1.4 : HOLD_SECONDS;
        double direction = run.forward ? 1.0 : -1.0;
        Pose velocity = context.getFollower().getVelocity();
        double measured = direction * (angular ? velocity.getHeading().getRad() : velocity.getX().getIn());
        Pose acceleration = context.getFollower().getAcceleration();
        double measuredAcceleration = direction * (angular ? acceleration.getHeading().getRad()
                : acceleration.getX().getIn());
        if (elapsed >= duration) {
            if (!validating && holdSamples >= 10) {
                (angular ? angularHolds : translationHolds).add(new HoldingPoint(
                        holdVelocitySum / holdSamples, holdPowerSum / holdSamples));
            }
            context.getFollower().stop();
            timer.reset();
            autoStage = AutoStage.SETTLING;
            return;
        }

        double dt = Math.max(0, Math.min(.1, elapsed - holdLastTime));
        holdLastTime = elapsed;
        double target;
        double power;
        if (validating) {
            double[] fit = angular ? angularCandidate : translationCandidate;
            double rampTime = .7;
            target = .4 * limit * Math.min(1, elapsed / rampTime);
            double targetAcceleration = elapsed < rampTime ? .4 * limit / rampTime : 0;
            power = fit[0] + fit[1] * target + fit[2] * targetAcceleration;
            int index = autoRunIndex - VALIDATION_START;
            validationSaturated[index] |= !Double.isFinite(power) || Math.abs(power) > 1.0;
            boolean sample = elapsed >= .2;
            if (sample) {
                double error = measured - target;
                validationSquared[index] += error * error;
                validationSamples[index]++;
                int phase = elapsed < rampTime ? 0 : 1;
                validationBias[index][phase] += error;
                validationPhaseSquared[index][phase] += error * error;
                validationPhaseSamples[index][phase]++;
            }
            validationCsv.writeRow(autoRunIndex, run.axis, run.forward ? "FORWARD" : "REVERSE",
                    elapsed, target, measured, power, sample);
        } else {
            target = run.speedFraction * limit;
            double kp = 3.0 / Math.max(limit, 1e-6);
            double ki = 9.0 / Math.max(limit, 1e-6);
            double error = target - measured;
            double seed = angular ? context.constants.angularCoeffs.kS : context.constants.translationalCoeffs.kS;
            double candidateIntegral = holdIntegral + ki * error * dt;
            double requested = seed + kp * error + candidateIntegral;
            if (requested >= 0 && requested <= .85) { holdIntegral = candidateIntegral; }
            power = Math.max(0, Math.min(.85, seed + kp * error + holdIntegral));
            // Use the previous applied power: it produced this velocity/acceleration sample.
            if (elapsed >= .8 && Math.abs(error) <= .05 * limit &&
                    Math.abs(measuredAcceleration) <= .04 * limit && measured > .05 * limit) {
                holdVelocitySum += measured;
                holdPowerSum += lastAppliedCharacterizationPower;
                holdSamples++;
            }
        }
        context.getFollower().getDrivetrain().moveWithVectors(
                angular ? 0 : direction * clipManualPower(power), 0,
                angular ? direction * clipManualPower(power) : 0);
        lastAppliedCharacterizationPower = power;
    }

    /** Starts a fresh derivative-free integration window. */
    private void resetIntegrationWindow() {
        windowSignTime = 0.0;
        windowDistance = 0.0;
        windowVelocityChange = 0.0;
        windowPowerTime = 0.0;
        integrationWindowStarted = false;
        previousIntegrationTime = Double.NaN;
    }

    /** Accumulates one aligned command/velocity sample and emits a 0.2-second model window. */
    private void recordIntegralSample(AutoRun run, double directedPower,
                                      double rawVelocity, double elapsed) {
        double velocity = rawVelocity * (run.forward ? 1.0 : -1.0);
        double velocityFloor = run.axis == Axis.ANGULAR ? 0.02 : 0.25;
        if (!Double.isFinite(velocity) || velocity < velocityFloor) {
            resetIntegrationWindow();
            previousIntegrationTime = elapsed;
            return;
        }
        if (!Double.isFinite(previousIntegrationTime) || !integrationWindowStarted) {
            windowStartVelocity = velocity;
            previousWindowVelocity = velocity;
            previousIntegrationTime = elapsed;
            integrationWindowStarted = true;
            return;
        }

        double dt = elapsed - previousIntegrationTime;
        previousIntegrationTime = elapsed;
        if (dt <= 0.0 || dt > 0.10) {
            resetIntegrationWindow();
            previousIntegrationTime = elapsed;
            return;
        }

        windowPowerTime += directedPower * dt;
        windowSignTime += dt;
        windowDistance += 0.5 * (previousWindowVelocity + velocity) * dt;
        windowVelocityChange = velocity - windowStartVelocity;
        previousWindowVelocity = velocity;

        if (windowSignTime >= INTEGRATION_WINDOW_SECONDS) {
            IntegralObservation observation = new IntegralObservation(
                    windowSignTime, windowDistance, windowVelocityChange,
                    windowPowerTime);
            (run.axis == Axis.ANGULAR
                    ? angularIntegralObservations : translationalIntegralObservations)
                    .add(observation);
            resetIntegrationWindow();
            previousIntegrationTime = elapsed;
            windowStartVelocity = velocity;
            previousWindowVelocity = velocity;
            integrationWindowStarted = true;
        }
    }

    private boolean usesMovingAverageVelocity() {
        return context.getFollower().getLocalizer().getVelocityFilterMode()
                == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE;
    }

    private String feedforwardActionDescription(AutoRun run) {
        if (autoStage == AutoStage.SETTLING) {
            return "Robot is stopping before the next feedforward run.";
        }
        String motion;
        if (run.axis == Axis.ANGULAR) {
            motion = run.forward ? "turning counterclockwise" : "turning clockwise";
        } else {
            motion = run.forward ? "driving forward" : "driving backward";
        }
        return "Robot is " + motion + " for the feedforward test.";
    }

    private void reportFitDiagnostics() {
        context.getTelemetry().addData("Active-fit evidence angular holds / accel windows", angularHolds.size() + " / " + angularIntegralObservations.size());
        context.getTelemetry().addData("Active-fit evidence translation holds / accel windows", translationHolds.size() + " / " + translationalIntegralObservations.size());
        context.getTelemetry().addData("Candidate angular kS / kV / kA", Arrays.toString(angularCandidate));
        context.getTelemetry().addData("Candidate translation kS / kV / kA", Arrays.toString(translationCandidate));
        for (int i = 0; i < validationSamples.length; i++) {
            for (int phase = 0; phase < 2; phase++) {
                int count = validationPhaseSamples[i][phase];
                double bias = count == 0 ? Double.NaN : validationBias[i][phase] / count;
                double rms = count == 0 ? Double.NaN : Math.sqrt(validationPhaseSquared[i][phase] / count);
                context.getTelemetry().addData("Validation " + (i + 1) + (phase == 0 ? " ramp RMS / bias" : " hold RMS / bias"), rms + " / " + bias);
            }
            context.getTelemetry().addData("Validation " + (i + 1) + " samples / saturated", validationSamples[i] + " / " + validationSaturated[i]);
        }
        context.getTelemetry().addData("Fit evidence CSV", csvPath);
        if (validationCsv != null) { context.getTelemetry().addData("Validation CSV", validationCsv.getPath()); }
    }

    /** Writes only evidence used by the active separated fit. */
    private void writeFitEvidenceCsv() {
        TuningCsvWriter writer = TuningCsvWriter.open("feedforward_fit_evidence",
                "axis", "evidence", "velocity_or_sign_time", "power_or_distance",
                "velocity_change", "power_time", "candidate_kS", "candidate_kV",
                "candidate_kA", "residual", "validation");
        writeFitEvidenceAxis(writer, "ANGULAR", angularHolds, angularIntegralObservations, angularCandidate);
        writeFitEvidenceAxis(writer, "TRANSLATIONAL", translationHolds, translationalIntegralObservations, translationCandidate);
        writer.close();
        csvPath = writer.getPath();
        csvError = writer.getError();
    }

    private void writeFitEvidenceAxis(TuningCsvWriter writer, String axis, List<HoldingPoint> holds,
            List<IntegralObservation> windows, double[] candidate) {
        for (HoldingPoint point : holds) {
            double prediction = candidate[0] + candidate[1] * point.velocity;
            writer.writeRow(axis, "HOLD", point.velocity, point.power, Double.NaN, Double.NaN,
                    candidate[0], candidate[1], candidate[2], point.power - prediction, "");
        }
        for (IntegralObservation window : windows) {
            double prediction = candidate[0] * window.signTime + candidate[1] * window.distance + candidate[2] * window.velocityChange;
            writer.writeRow(axis, "ACCELERATION_WINDOW", window.signTime, window.distance,
                    window.velocityChange, window.powerTime, candidate[0], candidate[1],
                    candidate[2], window.powerTime - prediction, "");
        }
        writer.writeRow(axis, "RESULT", Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                candidate[0], candidate[1], candidate[2], Double.NaN, validationMessage);
    }
    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Angular KV", number(context.constants.angularKV));
        context.getTelemetry().addData("Angular KA", number(context.constants.angularKA));
        context.getTelemetry().addData("Translational KV", number(context.constants.translationalKV));
        context.getTelemetry().addData("Translational KA", number(context.constants.translationalKA));
        context.getTelemetry().addData("Validation", validationMessage);
        if (manualMode) {
            context.getTelemetry().addData("Drive kV cruise RMS", number(manualDriveMetrics.cruiseRms()));
            context.getTelemetry().addData("Angular kV cruise RMS", number(manualAngularMetrics.cruiseRms()));
            context.getTelemetry().addData("Drive kA whole-run RMS", number(manualDriveMetrics.wholeRunRms()));
            context.getTelemetry().addData("Angular kA whole-run RMS", number(manualAngularMetrics.wholeRunRms()));
            context.getTelemetry().addData("Manual response CSV", manualCsvPath);
            return;
        }
        reportFitDiagnostics();
        if (!context.isDebugMode()) { return; }
        context.getTelemetry().addData("Characterization CSV", csvPath);
        if (csvError != null) {
            context.getTelemetry().addData("CSV warning", csvError);
        }
    }
}
