package core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.EnumMap;

import drivetrains.BaseDrivetrain;
import geometry.Pose;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;

/**
 * Owns the shared or mode-specific localizers configured for one robot.
 *
 * <p>Mode-specific instances are built lazily. Switching preserves the last field pose and clears
 * the incoming estimator's derivative history through {@link BaseLocalizer#setPose(Pose)}.</p>
 */
public final class LocalizerSet {
    private final HardwareMap hardwareMap;
    private final LocalizationConstants calibration;
    private final boolean dualActuated;
    private final boolean shared;
    private final EnumMap<FollowerConstants.Profile, BaseLocalizerConstants<?>> configs =
            new EnumMap<>(FollowerConstants.Profile.class);
    private final EnumMap<FollowerConstants.Profile, BaseLocalizer<?>> localizers =
            new EnumMap<>(FollowerConstants.Profile.class);
    private FollowerConstants.Profile activeProfile;

    public LocalizerSet(ApexConstants robot, HardwareMap hardwareMap,
                        BaseDrivetrain.DrivetrainType drivetrainType,
                        FollowerConstants.Profile initialProfile) {
        this(robot, hardwareMap, drivetrainType, initialProfile, LocalizationConstants.load());
    }

    public LocalizerSet(ApexConstants robot, HardwareMap hardwareMap,
                        BaseDrivetrain.DrivetrainType drivetrainType,
                        FollowerConstants.Profile initialProfile,
                        LocalizationConstants calibration) {
        this.hardwareMap = hardwareMap;
        this.calibration = calibration;
        dualActuated = drivetrainType == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED;
        shared = !dualActuated || robot.usesSharedLocalizer();
        if (shared) {
            BaseLocalizerConstants<?> config = robot.localizerConstants();
            configs.put(FollowerConstants.Profile.DEFAULT, config);
            activeProfile = FollowerConstants.Profile.DEFAULT;
        } else {
            requireDualProfile(initialProfile);
            configs.put(FollowerConstants.Profile.TANK,
                    robot.localizerConstants(FollowerConstants.Profile.TANK));
            configs.put(FollowerConstants.Profile.HOLONOMIC,
                    robot.localizerConstants(FollowerConstants.Profile.HOLONOMIC));
            activeProfile = initialProfile;
        }
        buildActive(Pose.zero(), false, true);
    }

    /** Returns the active localizer. */
    public BaseLocalizer<?> getLocalizer() { return localizers.get(storageProfile(activeProfile)); }

    /** Returns the active configuration after compatible saved geometry was applied. */
    public BaseLocalizerConstants<?> getConfig() { return configs.get(storageProfile(activeProfile)); }

    /** Returns the active physical drivetrain profile. */
    public FollowerConstants.Profile getActiveProfile() { return activeProfile; }

    /** Returns the persistence key for the active setup. */
    public String getSetupName() { return storageProfile(activeProfile).name(); }

    /** Returns whether both drivetrain modes intentionally share one localizer instance. */
    public boolean isShared() { return shared; }

    /** Returns the calibration object shared with the tuner or runtime owner. */
    public LocalizationConstants getCalibration() { return calibration; }

    /** Selects a drivetrain mode, lazily building its localizer and preserving field pose. */
    public void select(FollowerConstants.Profile profile) {
        if (!dualActuated) { return; }
        requireDualProfile(profile);
        if (profile == activeProfile) { return; }
        Pose pose = getLocalizer().getPose();
        activeProfile = profile;
        if (!shared && !localizers.containsKey(profile)) { buildActive(pose, true, true); }
        else { getLocalizer().setPose(pose); }
    }

    /** Reconstructs the active localizer after accepted geometry changed. */
    public void rebuildActive() {
        Pose pose = getLocalizer().getPose();
        localizers.remove(storageProfile(activeProfile));
        buildActive(pose, true, false);
    }

    private void buildActive(Pose pose, boolean restorePose, boolean applySavedCalibration) {
        FollowerConstants.Profile key = storageProfile(activeProfile);
        BaseLocalizerConstants<?> config = configs.get(key);
        if (config == null) { throw new IllegalStateException("Missing localizer configuration for " + key); }
        if (applySavedCalibration) { calibration.applyGeometry(key.name(), config); }
        BaseLocalizer<?> localizer = config.build(hardwareMap);
        calibration.applyFilter(key.name(), config, localizer);
        if (restorePose) { localizer.setPose(pose); }
        localizers.put(key, localizer);
    }

    private FollowerConstants.Profile storageProfile(FollowerConstants.Profile profile) {
        return shared ? FollowerConstants.Profile.DEFAULT : profile;
    }

    private static void requireDualProfile(FollowerConstants.Profile profile) {
        if (profile != FollowerConstants.Profile.TANK
                && profile != FollowerConstants.Profile.HOLONOMIC) {
            throw new IllegalArgumentException("Dual-actuated localizers require TANK or HOLONOMIC");
        }
    }
}
