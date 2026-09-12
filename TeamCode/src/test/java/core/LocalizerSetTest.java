package core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import drivetrains.BaseDrivetrain;
import drivetrains.BaseDrivetrainConstants;
import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;

public class LocalizerSetTest {
    @Test public void sharedDualModesReuseLocalizerAndPreservePose() {
        Config shared = new Config("shared");
        ApexConstants robot = robot(true, shared, null, null);
        LocalizerSet set = new LocalizerSet(robot, null,
                BaseDrivetrain.DrivetrainType.DUAL_ACTUATED,
                FollowerConstants.Profile.HOLONOMIC, LocalizationConstants.empty());
        BaseLocalizer<?> initial = set.getLocalizer();
        Pose pose = pose(4, 5, 0.7);
        initial.setPose(pose);
        set.select(FollowerConstants.Profile.TANK);
        assertSame(initial, set.getLocalizer());
        assertEquals(pose, set.getLocalizer().getPose());
        assertEquals("DEFAULT", set.getSetupName());
    }

    @Test public void separateDualModesBuildIndependentLocalizersLazily() {
        Config tank = new Config("tank");
        Config holonomic = new Config("holonomic");
        ApexConstants robot = robot(false, holonomic, tank, holonomic);
        LocalizerSet set = new LocalizerSet(robot, null,
                BaseDrivetrain.DrivetrainType.DUAL_ACTUATED,
                FollowerConstants.Profile.HOLONOMIC, LocalizationConstants.empty());
        BaseLocalizer<?> first = set.getLocalizer();
        first.setPose(pose(7, -2, 1.2));
        set.select(FollowerConstants.Profile.TANK);
        assertNotSame(first, set.getLocalizer());
        assertEquals("tank", ((Config) set.getConfig()).id);
        assertEquals(7.0, set.getLocalizer().getPose().getX().getIn(), 0.0);
        assertEquals(-2.0, set.getLocalizer().getPose().getY().getIn(), 0.0);
        assertEquals(1.2, set.getLocalizer().getPose().getHeading().getRad(), 0.0);
        assertEquals("TANK", set.getSetupName());
    }

    private static ApexConstants robot(boolean shared, Config defaultConfig,
                                       Config tank, Config holonomic) {
        return new ApexConstants() {
            @Override public BaseDrivetrainConstants<?> drivetrainConstants() { return null; }
            @Override public BaseLocalizerConstants<?> localizerConstants() { return defaultConfig; }
            @Override public boolean usesSharedLocalizer() { return shared; }
            @Override public BaseLocalizerConstants<?> localizerConstants(
                    FollowerConstants.Profile profile) {
                return profile == FollowerConstants.Profile.TANK ? tank : holonomic;
            }
        };
    }

    private static Pose pose(double x, double y, double heading) {
        return new Pose(Vector.of(x, y, DistUnit.IN), Angle.fromRad(heading));
    }

    private static final class Config implements BaseLocalizerConstants<Config> {
        final String id;
        Config(String id) { this.id = id; }
        @Override public BaseLocalizer<?> build(HardwareMap hardwareMap) { return new Localizer(this); }
    }

    private static final class Localizer extends BaseLocalizer<Config> {
        Localizer(Config config) { super(config); }
        @Override public void update() { calculate(UpdateType.BOTH); }
        @Override public void setPose(Pose pose) { resetKinematicEstimate(pose); }
    }
}
