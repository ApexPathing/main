package org.firstinspires.ftc.teamcode.sim;

import core.*;
import drivetrains.*;
import geometry.*;
import localizers.*;
import org.json.JSONObject;
import org.junit.Test;
import paths.builders.TankPathBuilder;
import paths.builders.TurnBuilder;
import paths.movements.Path;
import paths.movements.Turn;
import java.lang.reflect.Field;
import static org.junit.Assert.*;

/** Uses simulated hardware only as motor/sensor I/O; does not claim tank physics coverage. */
public class DualActuatedProfilesTest {
    @Test public void modeSwitchRefreshesGainsAndTurnBuildRetainsIntendedMode() throws Exception {
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object prior = singleton.get(null);
        String priorStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        String priorUnlock = System.getProperty("apex.simulation.unlockTunerPhases");
        try {
            JSONObject tank = values(0.02, 25);
            JSONObject holo = values(0.04, 55);
            FollowerConstants constants = FollowerConstants.fromJson(new JSONObject()
                    .put("schemaVersion", 2).put("drivetrainType", "DUAL_ACTUATED")
                    .put("profiles", new JSONObject().put("TANK", tank).put("HOLONOMIC", holo)));
            singleton.set(null, constants);
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            DualActuated.Constants driveConfig = new DualActuated.Constants()
                    .setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                    .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR))
                    .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                    .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR))
                    .setTransitionSeconds(0);
            ApexConstants config = new ApexConstants() {
                public BaseDrivetrainConstants<?> drivetrainConstants() { return driveConfig; }
                public BaseLocalizerConstants<?> localizerConstants() {
                    return new Pinpoint.Constants().setName(ApexSimulation.PINPOINT);
                }
            };
            Follower follower = new Follower(config, hardware.hardwareMap);
            DualActuated drive = (DualActuated) follower.getDrivetrain();
            assertEquals(0.04, cached(follower, "translationalKV"), 0);
            Path tankPath = new TankPathBuilder(Pose.zero(),
                    new Pose(Vector.of(24, 0, DistUnit.IN), Angle.zero())).quickBuild();
            assertEquals(FollowerConstants.Profile.HOLONOMIC, constants.getActiveProfile());
            assertNotNull(tankPath.getFeedforwardLut());
            follower.follow(tankPath);
            assertFalse(drive.isHolonomic());
            assertEquals(0.02, cached(follower, "translationalKV"), 0);
            assertEquals(25, constants.forwardVelLimitIn, 0);
            follower.stop();
            follower.setPose(Pose.zero());
            Path reversePath = new TankPathBuilder(Pose.zero(),
                    new Pose(Vector.of(-24, 0, DistUnit.IN), Angle.zero()))
                    .interpolateWith(paths.heading.InterpolationStyle.TANGENT_BACKWARD).quickBuild();
            follower.follow(reversePath);
            follower.update(false);
            assertTrue(follower.getTrackingVelocityTarget() < 0);
            follower.stop();
            Turn turn = new TurnBuilder(Pose.zero())
                    .setDriveProfile(FollowerConstants.Profile.HOLONOMIC)
                    .turnTo(Angle.fromDeg(90)).profiledBuild();
            assertEquals(FollowerConstants.Profile.TANK, constants.getActiveProfile());
            follower.follow(turn.reversed());
            assertTrue(drive.isHolonomic());
            assertEquals(0.04, cached(follower, "translationalKV"), 0);
            follower.stop();

            driveConfig.setTransitionSeconds(1);
            drive.activateTractionState();
            drive.setPowers(1, 1, 1, 1);
            assertTrue(drive.isTransitioning());
            assertEquals(0, drive.getLastFlPower(), 0);
            follower.update(false);
            assertEquals(FollowerConstants.Profile.TANK, constants.getActiveProfile());
            assertEquals(0.02, cached(follower, "translationalKV"), 0);
        } finally {
            singleton.set(null, prior);
            restore(ApexStorage.DIRECTORY_PROPERTY, priorStorage);
            restore("apex.simulation.unlockTunerPhases", priorUnlock);
        }
    }

    private static JSONObject values(double kv, double velocity) throws Exception {
        return new JSONObject().put("translationKV", kv).put("translationKA", .003)
                .put("angularKV", .1).put("angularKA", .01)
                .put("headingP", .5).put("translationalP", .2)
                .put("forwardVelLimitIn", velocity).put("forwardAccelLimitIn", 40)
                .put("strafeVelLimitIn", 30).put("strafeAccelLimitIn", 30)
                .put("angularVelLimitRad", 3).put("angularAccelLimitRad", 5);
    }
    private static double cached(Follower follower, String name) throws Exception {
        Field field = Follower.class.getDeclaredField(name);
        field.setAccessible(true);
        return field.getDouble(follower);
    }
    private static void restore(String key, String value) {
        if (value == null) { System.clearProperty(key); } else { System.setProperty(key, value); }
    }
}
