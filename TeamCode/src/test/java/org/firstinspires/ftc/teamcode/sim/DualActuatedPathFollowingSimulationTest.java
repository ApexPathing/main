package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.json.JSONObject;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;

import core.ApexConstants;
import core.Follower;
import core.FollowerConstants;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.DualActuated;
import drivetrains.Motor;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import localizers.BaseLocalizerConstants;
import localizers.Pinpoint;
import paths.heading.InterpolationStyle;
import paths.movements.FollowerMovement;
import paths.movements.Path;

/** End-to-end path following regression that changes a dual-actuated drive between both modes. */
public class DualActuatedPathFollowingSimulationTest {
    private static final String DRIVE_MODE_SERVO = "driveMode";
    private static final double TANK_SERVO_POSITION = 0.85;
    private static final double HOLONOMIC_SERVO_POSITION = 0.10;
    private static final double POSITION_TOLERANCE_IN = 3.0;
    private static final double HEADING_TOLERANCE_DEG = 5.0;

    @Test(timeout = 50_000L)
    public void followsHolonomicThenTankThenHolonomicPaths() throws Exception {
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object prior = singleton.get(null);
        try {
            singleton.set(null, dualFollowerConstants());
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            double[] servoPosition = {Double.NaN};
            hardware.hardwareMap.register(DRIVE_MODE_SERVO, simulatedServo(servoPosition));
            Follower follower = new Follower(new DualSimulationConstants(), hardware.hardwareMap);
            DualActuated drivetrain = (DualActuated) follower.getDrivetrain();
            GeometryFactory factory = new GeometryFactory(follower)
                    .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);

            Pose start = factory.pose(0, 0, 0);
            Path strafe = factory.holonomicPath(start, factory.pose(18, 12, 0))
                    .profiledBuild();
            Path traction = factory.tankPath(strafe.getEndPose(), factory.pose(30, 12),
                            factory.pose(42, 24, 45))
                    .interpolateWith(InterpolationStyle.TANGENT_FORWARD).profiledBuild();
            Path strafeAgain = factory.holonomicPath(
                            traction.getEndPose(), factory.pose(42, 8, 45))
                    .profiledBuild();

            follower.setPose(start);
            assertTrue(drivetrain.isHolonomic());
            assertEquals(HOLONOMIC_SERVO_POSITION, servoPosition[0], 0.0);
            follow(hardware, follower, strafe, 12.0);

            follower.follow(traction);
            assertFalse(drivetrain.isHolonomic());
            assertEquals(TANK_SERVO_POSITION, servoPosition[0], 0.0);
            assertTrue("A nonzero transition should be observable", drivetrain.isTransitioning());
            follower.update();
            assertEquals(0.0, drivetrain.getLastFlPower(), 0.0);
            assertEquals(0.0, drivetrain.getLastFrPower(), 0.0);
            followAlreadyStarted(hardware, follower, traction, 12.0);

            follow(hardware, follower, strafeAgain, 12.0);
            assertTrue(drivetrain.isHolonomic());
            assertEquals(HOLONOMIC_SERVO_POSITION, servoPosition[0], 0.0);
        } finally {
            singleton.set(null, prior);
        }
    }

    private static void follow(ApexSimulation.Hardware hardware, Follower follower,
                               FollowerMovement movement, double timeoutSeconds) throws Exception {
        follower.follow(movement);
        followAlreadyStarted(hardware, follower, movement, timeoutSeconds);
    }

    private static void followAlreadyStarted(ApexSimulation.Hardware hardware, Follower follower,
                                             FollowerMovement movement, double timeoutSeconds)
            throws Exception {
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
        assertFalse("Dual-actuated movement timed out: pose=" + actual, follower.isBusy());
        assertTrue("Endpoint position error was " + positionError + " in at " + actual,
                positionError <= POSITION_TOLERANCE_IN);
        assertTrue("Endpoint heading error was " + headingError + " deg at " + actual,
                headingError <= HEADING_TOLERANCE_DEG);
    }

    private static FollowerConstants dualFollowerConstants() throws Exception {
        JSONObject shared = new JSONObject()
                .put("headingP", 3.130238282794052)
                .put("headingD", 0.44422514288504944).put("headingS", 0.23125)
                .put("translationalP", 0.11877226292167513)
                .put("translationalD", 0.028499409724280098)
                .put("translationalS", 0.23125)
                .put("translationKV", 0.0071221715453131966)
                .put("translationKA", 0.004660337129387047)
                .put("angularKV", 0.06566610479427347)
                .put("angularKA", 0.04305298771810743)
                .put("velocityFeedbackGain", 0.05926269306719753)
                .put("angularVelocityFeedbackGain", 1.664473280244918)
                .put("kCentripetal", 0.006116371767550798)
                .put("forwardVelLimitIn", 64.67997142368108)
                .put("forwardAccelLimitIn", 105.65980741005704)
                .put("strafeVelLimitIn", 53.6).put("strafeAccelLimitIn", 94.8)
                .put("angularVelLimitRad", 6.96).put("angularAccelLimitRad", 14.66);
        return FollowerConstants.fromJson(new JSONObject().put("schemaVersion", 2)
                .put("drivetrainType", "DUAL_ACTUATED")
                .put("profiles", new JSONObject()
                        .put("TANK", new JSONObject(shared.toString()))
                        .put("HOLONOMIC", new JSONObject(shared.toString()))));
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

    private static Servo simulatedServo(double[] position) {
        return (Servo) Proxy.newProxyInstance(
                DualActuatedPathFollowingSimulationTest.class.getClassLoader(),
                new Class<?>[] {Servo.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPosition")) {
                        position[0] = ((Number) args[0]).doubleValue();
                        return null;
                    }
                    if (method.getName().equals("getPosition")) { return position[0]; }
                    if (method.getName().equals("getManufacturer")) {
                        return HardwareDevice.Manufacturer.Other;
                    }
                    if (method.getName().equals("getDeviceName")) { return "Simulated Servo"; }
                    if (method.getName().equals("getVersion")) { return 1; }
                    if (method.getName().equals("getConnectionInfo")) { return "simulated"; }
                    Class<?> type = method.getReturnType();
                    if (!type.isPrimitive()) { return null; }
                    if (type == boolean.class) { return false; }
                    if (type == char.class) { return '\0'; }
                    return 0;
                });
    }

    private static final class DualSimulationConstants implements ApexConstants {
        @Override public BaseDrivetrainConstants<?> drivetrainConstants() {
            return new DualActuated.Constants()
                    .setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                    .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR).reverse())
                    .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                    .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR).reverse())
                    .setInitialState(DualActuated.DriveState.HOLONOMIC)
                    .setTransitionSeconds(0.08)
                    .addActuator(DRIVE_MODE_SERVO, TANK_SERVO_POSITION,
                            HOLONOMIC_SERVO_POSITION)
                    .setMaxPower(1.0);
        }

        @Override public BaseLocalizerConstants<?> localizerConstants() {
            return new Pinpoint.Constants().setName(ApexSimulation.PINPOINT);
        }
    }
}
