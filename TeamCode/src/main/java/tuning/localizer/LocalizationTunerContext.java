package tuning.localizer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.json.JSONObject;

import core.ApexConstants;
import core.FollowerConstants;
import core.LocalizationConstants;
import core.LocalizerSet;
import drivetrains.BaseDrivetrain;
import drivetrains.DualActuated;
import geometry.Pose;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;

/** Shared hardware, persistence, and drive control used by localization-tuner phases. */
public final class LocalizationTunerContext extends tuning.TunerContext {
    private final BaseDrivetrain<?> drivetrain;
    private final LocalizerSet localizers;
    private LocalizerAdapter adapter;
    private String lastSaveError = "";

    public LocalizationTunerContext(LinearOpMode opMode, ApexConstants robot) {
        super(opMode);
        drivetrain = robot.drivetrainConstants().build(opMode.hardwareMap);
        FollowerConstants.Profile initial = activeDriveProfile();
        localizers = new LocalizerSet(robot, opMode.hardwareMap,
                drivetrain.getDrivetrainType(), initial, LocalizationConstants.load());
        adapter = LocalizerAdapters.create(localizers.getConfig());
    }

    /** Updates the active hardware localizer exactly once for the current loop. */
    public void update() { localizers.getLocalizer().update(); }

    public BaseDrivetrain<?> getDrivetrain() { return drivetrain; }
    public BaseLocalizer<?> getLocalizer() { return localizers.getLocalizer(); }
    public BaseLocalizerConstants<?> getConfig() { return localizers.getConfig(); }
    public LocalizerAdapter getAdapter() { return adapter; }
    public LocalizationConstants getCalibration() { return localizers.getCalibration(); }
    public String getSetupName() { return localizers.getSetupName(); }
    public boolean isSharedLocalizer() { return localizers.isShared(); }
    public String getLastSaveError() { return lastSaveError; }

    /** Applies robot-centric operator input, optionally suppressing translation. */
    public void manualDrive(boolean allowTranslation) {
        double forward = allowTranslation ? -opMode.gamepad1.left_stick_y : 0.0;
        double strafe = allowTranslation && drivetrain.isHolonomic()
                ? -opMode.gamepad1.left_stick_x : 0.0;
        drivetrain.moveWithVectors(forward, strafe, -opMode.gamepad1.right_stick_x);
    }

    public void stop() { drivetrain.stop(); }

    /** Returns whether a DualActuated drivetrain is still moving its mode actuators. */
    public boolean isDriveTransitioning() {
        return drivetrain instanceof DualActuated && ((DualActuated) drivetrain).isTransitioning();
    }

    /** Switches a DualActuated drivetrain and selects its corresponding localizer setup. */
    public void toggleDualActuatedMode() {
        if (!(drivetrain instanceof DualActuated)) { return; }
        stop();
        DualActuated dual = (DualActuated) drivetrain;
        if (dual.isHolonomic()) { dual.activateTractionState(); }
        else { dual.activateHolonomicState(); }
        localizers.select(activeDriveProfile());
        adapter = LocalizerAdapters.create(localizers.getConfig());
    }

    /** Applies and immediately persists an operator-approved geometry candidate. */
    public boolean acceptGeometry(String step, CalibrationCandidate candidate) {
        JSONObject oldValues = copy(getConfig().getCalibrationValues());
        JSONObject oldCalibration = copy(localizers.getCalibration().toJson());
        try {
            getConfig().applyCalibrationValues(candidate.getValues());
            localizers.rebuildActive();
            LocalizationConstants.Status priorFilter = localizers.getCalibration().getStatus(
                    getSetupName(), true);
            localizers.getCalibration().capture(getSetupName(), getConfig(), getLocalizer(),
                    LocalizationConstants.Status.ACCEPTED,
                    priorFilter == LocalizationConstants.Status.UNCALIBRATED
                            ? priorFilter : LocalizationConstants.Status.NEEDS_VALIDATION);
            localizers.getCalibration().markGeometryStepAccepted(getSetupName(), step);
            localizers.getCalibration().save();
            lastSaveError = "";
            return true;
        } catch (Exception failure) {
            localizers.getCalibration().restore(oldCalibration);
            getConfig().applyCalibrationValues(oldValues);
            localizers.rebuildActive();
            lastSaveError = failure.getMessage();
            return false;
        }
    }

    /** Immediately persists the current velocity-filter settings. */
    public boolean acceptFilter() {
        JSONObject oldCalibration = copy(localizers.getCalibration().toJson());
        try {
            localizers.getCalibration().capture(getSetupName(), getConfig(), getLocalizer(),
                    localizers.getCalibration().getStatus(getSetupName(), false),
                    LocalizationConstants.Status.ACCEPTED);
            localizers.getCalibration().save();
            lastSaveError = "";
            return true;
        } catch (Exception failure) {
            localizers.getCalibration().restore(oldCalibration);
            lastSaveError = failure.getMessage();
            return false;
        }
    }

    /** Records that the operator reviewed a complete validation drive. */
    public boolean acceptValidation() {
        JSONObject oldCalibration = copy(localizers.getCalibration().toJson());
        try {
            LocalizationConstants.Status geometry = localizers.getCalibration().getStatus(
                    getSetupName(), false);
            LocalizationConstants.Status filter = localizers.getCalibration().getStatus(
                    getSetupName(), true);
            localizers.getCalibration().capture(getSetupName(), getConfig(), getLocalizer(),
                    geometry == LocalizationConstants.Status.UNCALIBRATED ? geometry
                            : LocalizationConstants.Status.ACCEPTED,
                    filter == LocalizationConstants.Status.UNCALIBRATED ? filter
                            : LocalizationConstants.Status.ACCEPTED);
            localizers.getCalibration().save();
            lastSaveError = "";
            return true;
        } catch (Exception failure) {
            localizers.getCalibration().restore(oldCalibration);
            lastSaveError = failure.getMessage();
            return false;
        }
    }

    @Override
    public void addInterfaceHeader() {
        getTelemetry().addData("Localizer", adapter.getName());
        if (drivetrain instanceof DualActuated) {
            getTelemetry().addData("Drive mode", activeDriveProfile());
            getTelemetry().addData("Localizer setup",
                    localizers.isShared() ? "SHARED" : getSetupName());
        }
        addDebugHeader();
        getTelemetry().addLine();
    }

    private FollowerConstants.Profile activeDriveProfile() {
        if (drivetrain instanceof DualActuated && !((DualActuated) drivetrain).isHolonomic()) {
            return FollowerConstants.Profile.TANK;
        }
        return drivetrain instanceof DualActuated
                ? FollowerConstants.Profile.HOLONOMIC : FollowerConstants.Profile.DEFAULT;
    }

    private static JSONObject copy(JSONObject source) {
        try { return new JSONObject(source.toString()); }
        catch (Exception e) { throw new IllegalStateException("Could not copy localizer values", e); }
    }
}
