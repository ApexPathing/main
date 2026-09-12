package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import java.nio.file.Files;
import java.util.Collections;
import java.util.ArrayList;
import java.util.List;
import core.ApexStorage;
import core.Follower;
import core.LocalizationConstants;
import controllers.PDSController.PDSCoefficients;
import tuning.follower.phases.FeedforwardTuner;
import tuning.follower.TunerContext;

/** Exercises ramp regression and acceptance through the real tuner UI. */
public class FeedforwardPhaseSimulationTest {
    private java.lang.reflect.Field singleton;
    private Object previousConstants;

    @org.junit.Before public void isolateConstants() throws Exception {
        singleton = core.FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        previousConstants = singleton.get(null);
        singleton.set(null, null);
    }

    @org.junit.After public void restoreConstants() throws Exception {
        singleton.set(null, previousConstants);
    }

    @Test
    public void acceptedLocalizationStatusIsAvailableToTuner() throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                Files.createTempDirectory("apex-feedforward-prerequisite-").toString());
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            Constants robot = new Constants();
            Follower untuned = new Follower(robot, hardware.hardwareMap, true);
            assertFalse(untuned.hasAcceptedKalmanFilterTuning());

            LocalizationConstants localization = LocalizationConstants.empty();
            localization.capture("DEFAULT", robot.localizerConstants(), untuned.getLocalizer(),
                    LocalizationConstants.Status.ACCEPTED,
                    LocalizationConstants.Status.ACCEPTED);
            localization.save();

            Follower tuned = new Follower(robot, hardware.hardwareMap, true);
            assertTrue(tuned.hasAcceptedKalmanFilterTuning());
        } finally {
            if (previousDirectory == null) {
                System.clearProperty(ApexStorage.DIRECTORY_PROPERTY);
            } else {
                System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory);
            }
        }
    }

    @Test(timeout = 150000)
    public void automaticFeedforwardRampFitsBothAxes() throws Exception {
        runRamp(false);
    }

    @Test(timeout = 150000)
    public void tankFeedforwardRampRecoversLowFrictionModel() throws Exception {
        runRamp(true);
    }

    private void runRamp(boolean tank) throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                Files.createTempDirectory("apex-feedforward-phase-").toString());
        SimLinearOpModeBridge.Session session = null;
        try {
            ApexSimulation.Hardware hardware = tank ? ApexSimulation.createTankHardware()
                    : ApexSimulation.createHardware();
            List<String> frames = Collections.synchronizedList(new ArrayList<>());
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            java.util.concurrent.atomic.AtomicReference<TunerContext> tunedContext = new java.util.concurrent.atomic.AtomicReference<>();
            LinearOpMode opMode = new LinearOpMode() {
                @Override public void runOpMode() throws InterruptedException {
                    core.ApexConstants robot = tank ? new TankSimulationConstants() : new Constants();
                    Follower seed = new Follower(robot, hardwareMap, true);
                    LocalizationConstants localization = LocalizationConstants.empty();
                    localization.capture("DEFAULT", robot.localizerConstants(), seed.getLocalizer(),
                            LocalizationConstants.Status.ACCEPTED,
                            LocalizationConstants.Status.ACCEPTED);
                    try { localization.save(); }
                    catch (java.io.IOException e) { throw new IllegalStateException(e); }
                    TunerContext context = new TunerContext(this);
                    context.setFollower(new Follower(robot, hardwareMap, true));
                    context.constants.forwardVelLimitIn = 65;
                    context.constants.forwardAccelLimitIn = 148;
                    context.constants.angularVelLimitRad = 7;
                    context.constants.angularAccelLimitRad = 14.5;
                    context.constants.translationalCoeffs = new PDSCoefficients(.20, .04, .24);
                    context.constants.angularCoeffs = new PDSCoefficients(2.40, .45, .24);
                    context.getFollower().setDriveCoefficients(context.constants.translationalCoeffs);
                    context.getFollower().setHeadingCoefficients(context.constants.angularCoeffs);
                    if (tank) {
                        // This phase identifies kS/kV only; provide the independently known
                        // inertia and velocity-loop gains before testing its resulting fit.
                        context.getFollower().setFeedforwardGains(0.94 / 75, 0.94 / 150,
                                7 * 0.94 / 75, 7 * 0.94 / 150);
                        context.getFollower().setVelocityFeedback(0.02, 0.15);
                    }
                    waitForStart();
                    tunedContext.set(context);
                    new FeedforwardTuner(context).run(this);

                }
            };
            opMode.hardwareMap = hardware.hardwareMap;
            opMode.telemetry = telemetry;
            opMode.gamepad1 = new Gamepad();
            opMode.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(opMode, () -> {});
            SimLinearOpModeBridge.start(session);
            long deadline = System.nanoTime() + 130_000_000_000L;
            long previous = System.nanoTime();
            long lastPress = 0;
            String frame = "";
            while (System.nanoTime() < deadline) {
                long now = System.nanoTime();
                double remaining = Math.min(.05, (now - previous) * 1e-9);
                previous = now;
                while (remaining > 1e-9) {
                    double dt = Math.min(.005, remaining);
                    step(hardware, dt);
                    remaining -= dt;
                }
                frame = frames.isEmpty() ? "" : frames.get(frames.size() - 1);
                if (frame.contains("did not pass validation") || frame.contains("phase complete with results")) {
                    break;
                }
                if (frame.contains("Press A") && now - lastPress > 250_000_000L) {
                    lastPress = now;
                }
                opMode.gamepad1.a = now - lastPress < 60_000_000L;
                SimLinearOpModeBridge.eventLoopIteration(session, opMode.gamepad1, opMode.gamepad2);
                Thread.sleep(5);
            }
            System.out.println(frame);
            TunerContext context = tunedContext.get();
            if (tank) {
                org.junit.Assert.assertEquals(0.06,
                        context.constants.translationalFeedforwardKS, 0.005);
                org.junit.Assert.assertEquals(0.06,
                        context.constants.angularFeedforwardKS, 0.005);
                org.junit.Assert.assertEquals(0.94 / 75,
                        context.constants.translationalKV, 0.0005);
                org.junit.Assert.assertEquals(7 * 0.94 / 75,
                        context.constants.angularKV, 0.003);
            }
            assertTrue("Tuner failed to converge: " + frame,
                    frame.contains("phase complete with results") && frame.contains("kS/kV accepted"));
            if (tank) {
                SimLinearOpModeBridge.stop(session);
                session = null;
                Follower follower = context.getFollower();
                geometry.GeometryFactory factory = new geometry.GeometryFactory(follower)
                        .setDistUnit(geometry.DistUnit.IN).setAngleUnit(geometry.AngleUnit.DEG);
                geometry.Pose start = factory.pose(0, 0, 0);
                follower.setPose(start);
                paths.movements.Path curve = factory.tankPath(start, factory.pose(18, 0),
                                factory.pose(30, 6), factory.pose(36, 18, 45))
                        .interpolateWith(paths.heading.InterpolationStyle.TANGENT_FORWARD).profiledBuild();
                TankPathFollowingSimulationTest.follow(hardware, follower, curve, 15);
            }
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            if (previousDirectory == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory); }
        }
    }

    private static void step(ApexSimulation.Hardware hardware, double dt) throws Exception {
        double[] wheels = new double[hardware.drivetrain.motorNames.length];
        for (int i = 0; i < wheels.length; i++) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(DcMotorEx.class,
                    hardware.drivetrain.motorNames[i]);
            motor.update(dt);
            wheels[i] = motor.getVelocity();
        }
        java.lang.reflect.Method method = hardware.drivetrain.getClass()
                .getDeclaredMethod("forwardKinematics", double[].class);
        method.setAccessible(true);
        MotionVector velocity = (MotionVector) method.invoke(hardware.drivetrain, (Object) wheels);
        hardware.drivetrain.velocity = velocity.toFieldFrame(hardware.drivetrain.position.theta);
        hardware.drivetrain.position = hardware.drivetrain.position.step(hardware.drivetrain.velocity, dt);
    }
}
