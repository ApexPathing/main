package org.firstinspires.ftc.teamcode.sim;

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
import tuning.FeedforwardTuner;
import tuning.TunerContext;

/** Exercises collection, fitting and independent validation through the real tuner UI. */
public class FeedforwardPhaseSimulationTest {
    @Test(timeout = 150000)
    public void automaticFeedforwardPhasePassesIndependentValidation() throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                Files.createTempDirectory("apex-feedforward-phase-").toString());
        SimLinearOpModeBridge.Session session = null;
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            List<String> frames = Collections.synchronizedList(new ArrayList<>());
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            LinearOpMode opMode = new LinearOpMode() {
                @Override public void runOpMode() throws InterruptedException {
                    TunerContext context = new TunerContext(this);
                    context.setFollower(new Follower(new Constants(), hardwareMap, true));
                    context.constants.forwardVelLimitIn = 65;
                    context.constants.forwardAccelLimitIn = 148;
                    context.constants.angularVelLimitRad = 7;
                    context.constants.angularAccelLimitRad = 14.5;
                    context.constants.translationalCoeffs.kS = .24;
                    context.constants.angularCoeffs.kS = .24;
                    waitForStart();
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
            assertTrue("Tuner failed to validate: " + frame,
                    frame.contains("phase complete with results") && frame.contains("PASSED"));
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
