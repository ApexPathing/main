package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedTank;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.codeblooded.ftcodesim.simulator.OpModeRegister;
import org.codeblooded.ftcodesim.simulator.SimConfig;
import org.json.JSONObject;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.HashSet;
import java.util.Set;

import core.Follower;
import core.FollowerConstants;
import core.ApexConstants;
import drivetrains.BaseDrivetrain;
import drivetrains.Tank;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import paths.heading.InterpolationStyle;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/** End-to-end regression against FTCodeSim's differential-drive physics model. */
public class TankPathFollowingSimulationTest {
    @Test public void tankMovesAtLowPowerAndHasLinearSteadySpeed() throws Exception {
        for (double power : new double[] {0.12, 0.3, 0.6, -0.3}) {
            ApexSimulation.Hardware hardware = ApexSimulation.createTankHardware();
            Tank drivetrain = (Tank) new TankSimulationConstants().drivetrainConstants()
                    .build(hardware.hardwareMap);
            drivetrain.moveWithVectors(power, 0, 0);
            for (int i = 0; i < 1500; i++) { stepPhysics(hardware, 0.005); }
            double expected = (power - Math.copySign(0.06, power)) / (0.94 / 75);
            org.junit.Assert.assertEquals("Speed at power " + power, expected,
                    hardware.drivetrain.velocity.x, 0.2);
        }
    }

    @Test public void tankInteractiveConfigRegistersHardwareAndOpModes() {
        SimConfig config = ApexSimulation.createTankConfig();
        assertTrue(config.simHardwareMap.get(IMU.class, ApexSimulation.IMU) != null);
        assertTrue(config.simHardwareMap.get(DcMotorEx.class,
                ApexSimulation.FRONT_LEFT_MOTOR) != null);

        Set<Class<?>> registered = new HashSet<>();
        for (OpMode opMode : new OpModeRegister().getOpModes()) {
            registered.add(opMode.getClass());
        }
        assertTrue(registered.contains(TankLocalizationTuner.class));
        assertTrue(registered.contains(TankFollowerTuner.class));
        assertTrue(registered.contains(TankAutoTest.class));

        ApexConstants followerTunerConfig = new TankFollowerTuner().createConstants();
        ApexConstants localizerTunerConfig = new TankLocalizationTuner().createConstants();
        assertTrue(followerTunerConfig.drivetrainConstants().build(config.simHardwareMap)
                .getDrivetrainType() == BaseDrivetrain.DrivetrainType.TANK);
        assertTrue(localizerTunerConfig.drivetrainConstants().build(config.simHardwareMap)
                .getDrivetrainType() == BaseDrivetrain.DrivetrainType.TANK);
    }

    @Test public void positiveTankTurnMatchesApexHeadingConvention() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createTankHardware();
        Tank drivetrain = (Tank) new TankSimulationConstants().drivetrainConstants()
                .build(hardware.hardwareMap);
        drivetrain.moveWithVectors(0.0, 0.0, 0.5);
        for (int i = 0; i < 100; i++) { stepPhysics(hardware, 0.01); }
        assertTrue("Positive tank turn produced " + hardware.drivetrain.position.theta,
                hardware.drivetrain.position.theta > 0.0);
    }

    @Test(timeout = 70_000L)
    public void ramseteCorrectsLateralAndHeadingErrorOnStraightPath() throws Exception {
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object prior = singleton.get(null);
        try {
            singleton.set(null, tankFollowerConstants());
            ApexSimulation.Hardware hardware = ApexSimulation.createTankHardware();
            Follower follower = new Follower(new TankSimulationConstants(), hardware.hardwareMap);
            GeometryFactory factory = new GeometryFactory(follower)
                    .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
            Path path = factory.tankPath(factory.pose(0, 0, 0), factory.pose(60, 0, 0))
                    .interpolateWith(InterpolationStyle.TANGENT_FORWARD).profiledBuild();
            follower.setPose(factory.pose(0, 4, 10));
            follower.follow(path);
            follower.update();
            org.junit.Assert.assertEquals("Tank diagnostics must report the 4-inch lateral offset",
                    4.0, Math.abs(follower.getCrossTrackErrorIn()), 0.01);
            assertTrue("Straight path must still command corrective rotation",
                    follower.getTrackingAngularVelocityTarget() < -0.05);
            follower.stop();
            follow(hardware, follower, path, 15.0);
        } finally { singleton.set(null, prior); }
    }

    @Test(timeout = 70_000L)
    public void tunedTankFollowsCurveTurnAndReversePath() throws Exception {
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object prior = singleton.get(null);
        try {
            singleton.set(null, tankFollowerConstants());
            ApexSimulation.Hardware hardware = ApexSimulation.createTankHardware();
            assertTrue(hardware.drivetrain instanceof SimulatedTank);
            Follower follower = new Follower(new TankSimulationConstants(), hardware.hardwareMap);
            GeometryFactory factory = new GeometryFactory(follower)
                    .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
            Pose start = factory.pose(0, 0, 0);
            Path curve = factory.tankPath(start, factory.pose(18, 0), factory.pose(30, 6),
                            factory.pose(36, 18, 45))
                    .interpolateWith(InterpolationStyle.TANGENT_FORWARD).profiledBuild();
            Turn turn = factory.turn(curve.getEndPose())
                    .turnTo(factory.angle(-135)).profiledBuild();
            Path reverse = factory.tankPath(turn.getEndPose(), factory.pose(48, 30),
                            factory.pose(54, 36))
                    .interpolateWith(InterpolationStyle.TANGENT_BACKWARD).profiledBuild();

            follower.setPose(start);
            follow(hardware, follower, curve, 15.0);
            follow(hardware, follower, turn, 15.0);
            follow(hardware, follower, reverse, 15.0);
        } finally {
            singleton.set(null, prior);
        }
    }

    static void follow(ApexSimulation.Hardware hardware, Follower follower,
                               FollowerMovement movement, double timeoutSeconds) throws Exception {
        follower.follow(movement);
        long deadline = System.nanoTime() + (long) (timeoutSeconds * 1e9);
        while (follower.isBusy() && System.nanoTime() < deadline) {
            stepPhysics(hardware, 0.02);
            follower.update();
            Thread.sleep(20);
        }
        Pose actual = follower.getPose();
        double positionError = actual.distanceTo(movement.getEndPose()).getIn();
        double headingError = Math.abs(actual.getHeading()
                .getShortestAngleTo(movement.getEndPose().getHeading()).getDeg());
        assertFalse("Tank movement timed out: pose=" + actual, follower.isBusy());
        assertTrue("Tank endpoint position error was " + positionError + " in at " + actual,
                positionError <= 3.0);
        assertTrue("Tank endpoint heading error was " + headingError + " deg at " + actual,
                headingError <= 5.0);
    }

    private static FollowerConstants tankFollowerConstants() throws Exception {
        JSONObject values = new JSONObject()
                .put("headingP", 2.4).put("headingD", 0.45).put("headingS", 0.24)
                .put("translationalP", 0.20).put("translationalD", 0.04)
                .put("translationalS", 0.24)
                .put("translationKV", 0.0133).put("translationKA", 0.0069)
                .put("translationalFeedforwardS", 0.11)
                .put("angularKV", 0.061).put("angularKA", 0.03)
                .put("angularFeedforwardS", 0.24)
                .put("velocityFeedbackGain", 0.02)
                .put("angularVelocityFeedbackGain", 0.15)
                .put("forwardVelLimitIn", 20).put("forwardAccelLimitIn", 30)
                .put("angularVelLimitRad", 2).put("angularAccelLimitRad", 4);
        return FollowerConstants.fromJson(new JSONObject().put("schemaVersion", 2)
                .put("drivetrainType", "TANK")
                .put("profiles", new JSONObject().put("DEFAULT", values)));
    }

    private static void stepPhysics(ApexSimulation.Hardware hardware, double dt) throws Exception {
        double[] wheelVelocities = new double[hardware.drivetrain.motorNames.length];
        for (int i = 0; i < hardware.drivetrain.motorNames.length; i++) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(
                    DcMotorEx.class, hardware.drivetrain.motorNames[i]);
            motor.update(dt);
            wheelVelocities[i] = motor.getVelocity();
        }
        Method kinematics = hardware.drivetrain.getClass()
                .getDeclaredMethod("forwardKinematics", double[].class);
        kinematics.setAccessible(true);
        MotionVector robotVelocity = (MotionVector) kinematics.invoke(
                hardware.drivetrain, (Object) wheelVelocities);
        hardware.drivetrain.velocity = robotVelocity.toFieldFrame(
                hardware.drivetrain.position.theta);
        hardware.drivetrain.position = hardware.drivetrain.position.step(
                hardware.drivetrain.velocity, dt);
    }
}
