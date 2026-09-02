package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.firstinspires.ftc.teamcode.apexpathing.AutoTest;
import org.junit.Test;

import java.lang.reflect.Method;
import java.io.File;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import core.FollowerConstants;
import core.ApexStorage;
import controllers.PDSController.PDSCoefficients;
import paths.movements.Path;

public class AutoTestSimulationTest {
    private static final double MAX_TOTAL_MOVEMENT_SECONDS = 30.0;

    @Test(timeout = 130_000L)
    public void autoTestCompletesEveryMovement() throws Exception {
        configureStableConstants();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        AutoTest auto = new AutoTest();
        auto.hardwareMap = hardware.hardwareMap;
        auto.telemetry = telemetry;
        auto.gamepad1 = new Gamepad();
        auto.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(auto, () -> { });
        try {
            pump(session, auto, telemetry, hardware, 100);
            long initializationDeadline = System.nanoTime() + 10_000_000_000L;
            while (auto.getOutboundPath() == null &&
                    System.nanoTime() < initializationDeadline) {
                pump(session, auto, telemetry, hardware, 20);
            }
            Path outbound = auto.getOutboundPath();
            assertTrue("Auto Test did not finish building its outbound path", outbound != null);
            long routeStartedNanos = System.nanoTime();
            SimLinearOpModeBridge.start(session);
            long deadline = System.nanoTime() + 100_000_000_000L;
            while (!latest(frames).contains("Current check COMPLETE") &&
                    !latest(frames).contains("Current check FAILED")) {
                if (System.nanoTime() >= deadline) { break; }
                pump(session, auto, telemetry, hardware, 20);
            }

            String frame = latest(frames);
            double routeRunSeconds = (System.nanoTime() - routeStartedNanos) * 1e-9;
            assertFalse("Auto Test failed after its outbound path:\n" + frame,
                    frame.contains("Current check FAILED"));
            assertTrue("Auto Test did not complete every movement:\n" + frame,
                    frame.contains("Current check COMPLETE"));
            System.out.println("AUTO TEST COMMAND DEMAND: " + auto.getCommandDemandReport());
            System.out.println("AUTO TEST TIMING: " + auto.getMovementTimingReport());
            System.out.println("AUTO TEST EXTERNAL RUNTIME: " + routeRunSeconds + " s");
            System.out.println("AUTO TEST LOOP: " + auto.getAverageLoopMilliseconds() + " ms");
            assertTrue("Auto Test loop was not approximately 20 ms: " +
                            auto.getAverageLoopMilliseconds(),
                    auto.getAverageLoopMilliseconds() >= 17.0 &&
                            auto.getAverageLoopMilliseconds() <= 23.0);
            assertTrue("Auto Test following regressed: external=" + routeRunSeconds +
                            "s, internal=" + auto.getMovementTimingReport(),
                    routeRunSeconds <= MAX_TOTAL_MOVEMENT_SECONDS);
            if (outbound.isProfiled()) {
                File velocityCsv = new File(auto.getOutboundVelocityCsvPath());
                assertTrue("Profiled outbound velocity CSV was not created: path=" +
                                auto.getOutboundVelocityCsvPath() + ", error=" +
                                auto.getOutboundVelocityCsvError(),
                        velocityCsv.isFile() && velocityCsv.length() > 0L);
            }
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    private static void pump(SimLinearOpModeBridge.Session session, AutoTest auto,
                             ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware,
                             long milliseconds) throws Exception {
        long deadline = System.nanoTime() + milliseconds * 1_000_000L;
        long lastUpdate = System.nanoTime();
        while (System.nanoTime() < deadline) {
            long now = System.nanoTime();
            double remaining = Math.max(0.001, Math.min(0.05, (now - lastUpdate) * 1e-9));
            lastUpdate = now;
            while (remaining > 1e-9) {
                double dt = Math.min(0.005, remaining);
                stepPhysics(hardware, dt);
                remaining -= dt;
            }
            SimLinearOpModeBridge.eventLoopIteration(session, auto.gamepad1, auto.gamepad2);
            telemetry.update();
            Thread.sleep(5);
        }
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

    private static String latest(List<String> frames) {
        return frames.isEmpty() ? "" : frames.get(frames.size() - 1);
    }

    private static void configureStableConstants() {
        if (System.getProperty(ApexStorage.DIRECTORY_PROPERTY) == null) {
            File directory = new File(System.getProperty("user.dir"), "build/ftcodesim-data");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        }
        FollowerConstants constants = FollowerConstants.getInstance();
        constants.angularCoeffs = new PDSCoefficients(2.40, 0.45, 0.236);
        constants.translationalCoeffs = new PDSCoefficients(0.20, 0.04, 0.235);
        constants.angularKV = 0.03729620016302293;
        constants.angularKA = 0.05762017415132837;
        constants.angularFeedforwardKS = 0.2571326953727573;
        constants.translationalKV = 0.007985735155227602;
        constants.translationalKA = 0.006869366751688844;
        constants.translationalFeedforwardKS = 0.10800918318444208;
        constants.kCentripetal = 0.008830155935013819;
        constants.velocityFeedbackGain = 0.08273740189064561;
        constants.angularVelocityFeedbackGain = 0.25;
        constants.forwardVelLimitIn = 64.82715934849021;
        constants.forwardAccelLimitIn = 138.67321291770276;
        constants.strafeVelLimitIn = 53.709295445494945;
        constants.strafeAccelLimitIn = 107.71546803022714;
        constants.angularVelLimitRad = 6.980307090964637;
        constants.angularAccelLimitRad = 14.46018035928052;
    }
}
