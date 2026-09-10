package localizers;

import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import org.junit.Test;
import java.util.function.IntSupplier;
import static org.junit.Assert.*;

public class TankMecanumDriveEncodersTest {
    private static final double EPS = 1e-8;

    private static TankDriveEncoders.Constants config() {
        return new TankDriveEncoders.Constants().setIMUName("imu");
    }

    private static final class Rig {
        final int[] ticks;
        double yaw;
        final TankDriveEncoders localizer;
        Rig(TankDriveEncoders.Constants config, int... initialTicks) {
            ticks = initialTicks;
            IntSupplier[] inputs = new IntSupplier[ticks.length];
            for (int i = 0; i < ticks.length; i++) {
                final int index = i;
                inputs[i] = () -> ticks[index];
            }
            localizer = new TankDriveEncoders(config, inputs, () -> yaw);
        }
    }

    @Test public void unequalSideCountsScalesAndReversals() {
        Rig rig = new Rig(config().addLeftEncoder("l1", 10, false)
                .addLeftEncoder("l2", 20, true).addRightEncoder("r", 10, false),
                100, 200, 300);
        rig.ticks[0] += 80;
        rig.ticks[1] -= 240;
        rig.ticks[2] += 200;
        rig.localizer.update();
        assertEquals(15, rig.localizer.getPose().getX().getIn(), EPS);
        assertEquals(0, rig.localizer.getPose().getY().getIn(), EPS);
    }

    @Test public void quarterCircleUsesArcIntegration() {
        Rig rig = new Rig(config().addLeftEncoder("l", 10, false)
                .addRightEncoder("r", 10, false), 0, 0);
        rig.ticks[0] = 100;
        rig.ticks[1] = 200;
        rig.yaw = Math.PI / 2;
        rig.localizer.update();
        assertEquals(30 / Math.PI, rig.localizer.getPose().getX().getIn(), EPS);
        assertEquals(30 / Math.PI, rig.localizer.getPose().getY().getIn(), EPS);
    }

    @Test public void spinWrapAndResetPreserveContinuousHeading() {
        Rig rig = new Rig(config().addLeftEncoder("l", 10, false)
                .addRightEncoder("r", 10, false), 400, 500);
        rig.yaw = Math.toRadians(179);
        rig.localizer.setPose(new Pose(Vector.of(5, 6, DistUnit.IN), Angle.fromRad(1)));
        rig.yaw = Math.toRadians(-179);
        rig.ticks[0] -= 10;
        rig.ticks[1] += 10;
        rig.localizer.update();
        assertEquals(5, rig.localizer.getPose().getX().getIn(), EPS);
        assertEquals(6, rig.localizer.getPose().getY().getIn(), EPS);
        assertEquals(1 + Math.toRadians(2), rig.localizer.getPose().getHeading().getRad(), EPS);
        rig.localizer.setPose(Pose.zero());
        rig.localizer.update();
        assertEquals(0, rig.localizer.getPose().getX().getIn(), EPS);
        assertEquals(0, rig.localizer.getRawVel().getHeading().getRad(), EPS);
    }

    @Test public void reverseTravelAndEncoderRollover() {
        Rig rig = new Rig(config().addLeftEncoder("l", 10, false)
                .addRightEncoder("r", 10, false), Integer.MAX_VALUE - 5, Integer.MAX_VALUE - 5);
        rig.ticks[0] += 20;
        rig.ticks[1] += 20;
        rig.localizer.update();
        assertEquals(2, rig.localizer.getPose().getX().getIn(), EPS);
        rig.ticks[0] -= 30;
        rig.ticks[1] -= 30;
        rig.localizer.update();
        assertEquals(-1, rig.localizer.getPose().getX().getIn(), EPS);
    }

    @Test public void rejectsInvalidConfigurationBeforeHardwareAccess() {
        assertThrows(IllegalArgumentException.class, () -> config().build(null));
        assertThrows(IllegalArgumentException.class, () -> config()
                .addLeftEncoder("same", 10, false).addRightEncoder("same", 10, false).build(null));
        assertThrows(IllegalArgumentException.class, () -> config()
                .addLeftEncoder("l", Double.NaN, false).addRightEncoder("r", 10, false).build(null));
    }
}
