package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import controllers.PDSController.PDSCoefficients;
import core.ApexStorage;
import core.FollowerConstants;

/** End-to-end coverage for the automatic follower-tuning workflow. */
public class FollowerTunerTelemetryTest {
    @Test(timeout = 420_000L)
    public void automaticWorkflowCompletesFromStaticFrictionThroughFeedback() throws Exception {
        resetFollowerConstants();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            pump(session, tuner, telemetry, hardware, 100);
            tuner.gamepad1.b = true;
            pump(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.b = false;
            SimLinearOpModeBridge.start(session);

            long deadline = System.nanoTime() + 390_000_000_000L;
            while (!latestFrame(frames).contains("All follower tuning phases are complete") &&
                    System.nanoTime() < deadline) {
                String frame = latestFrame(frames);
                if (frame.contains("Press A")) {
                    tuner.gamepad1.a = true;
                    pump(session, tuner, telemetry, hardware, 30);
                    tuner.gamepad1.a = false;
                } else if (frame.contains("Press B to continue.")) {
                    tuner.gamepad1.b = true;
                    pump(session, tuner, telemetry, hardware, 30);
                    tuner.gamepad1.b = false;
                }
                pump(session, tuner, telemetry, hardware, 20);
            }

            assertTrue("Automatic follower tuning did not finish. Latest telemetry:\n" +
                            latestFrame(frames),
                    latestFrame(frames).contains("All follower tuning phases are complete"));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    private static void pump(SimLinearOpModeBridge.Session session, FollowerTuner tuner,
                             ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware,
                             long milliseconds) throws Exception {
        long deadline = System.nanoTime() + milliseconds * 1_000_000L;
        long lastUpdate = System.nanoTime();
        while (System.nanoTime() < deadline) {
            long now = System.nanoTime();
            double remaining = Math.max(0.001, Math.min(0.05, (now - lastUpdate) * 1e-9));
            lastUpdate = now;
            while (remaining > 1e-9) {
                double step = Math.min(0.005, remaining);
                stepPhysics(hardware, step);
                remaining -= step;
            }
            SimLinearOpModeBridge.eventLoopIteration(
                    session, tuner.gamepad1, tuner.gamepad2);
            telemetry.update();
            Thread.sleep(5);
        }
    }

    private static void stepPhysics(ApexSimulation.Hardware hardware, double seconds)
            throws Exception {
        double[] wheelVelocities = new double[hardware.drivetrain.motorNames.length];
        for (int i = 0; i < hardware.drivetrain.motorNames.length; i++) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(
                    DcMotorEx.class, hardware.drivetrain.motorNames[i]);
            motor.update(seconds);
            wheelVelocities[i] = motor.getVelocity();
        }
        java.lang.reflect.Method kinematics = hardware.drivetrain.getClass()
                .getDeclaredMethod("forwardKinematics", double[].class);
        kinematics.setAccessible(true);
        MotionVector robotVelocity = (MotionVector) kinematics.invoke(
                hardware.drivetrain, (Object) wheelVelocities);
        hardware.drivetrain.velocity = robotVelocity.toFieldFrame(
                hardware.drivetrain.position.theta);
        hardware.drivetrain.position = hardware.drivetrain.position.step(
                hardware.drivetrain.velocity, seconds);
    }

    private static String latestFrame(List<String> frames) {
        return frames.isEmpty() ? "" : frames.get(frames.size() - 1);
    }

    private static void resetFollowerConstants() {
        if (System.getProperty(ApexStorage.DIRECTORY_PROPERTY) == null) {
            File directory = new File(System.getProperty("user.dir"), "build/ftcodesim-data");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        }
        FollowerConstants constants = FollowerConstants.getInstance();
        constants.angularCoeffs = new PDSCoefficients();
        constants.translationalCoeffs = new PDSCoefficients();
        constants.angularKV = 0.0;
        constants.angularKA = 0.0;
        constants.translationalKV = 0.0;
        constants.translationalKA = 0.0;
        constants.kCentripetal = 0.0;
        constants.velocityFeedbackGain = 0.0;
        constants.angularVelocityFeedbackGain = 0.0;
        constants.forwardVelLimitIn = 0.0;
        constants.forwardAccelLimitIn = 0.0;
        constants.strafeVelLimitIn = 0.0;
        constants.strafeAccelLimitIn = 0.0;
        constants.angularVelLimitRad = 0.0;
        constants.angularAccelLimitRad = 0.0;
    }
}
