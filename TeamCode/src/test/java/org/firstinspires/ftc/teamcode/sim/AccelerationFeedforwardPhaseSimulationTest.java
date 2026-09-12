package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;

import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import core.ApexStorage;
import core.Follower;
import tuning.follower.TunerContext;
import tuning.follower.phases.AccelerationFeedforwardPhase;

import static org.junit.Assert.assertTrue;

/** End-to-end state-machine coverage for the one-button automatic kA search. */
public class AccelerationFeedforwardPhaseSimulationTest {
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

    @Test(timeout = 150000)
    public void automaticSearchTunesAndConfirmsBothAxes() throws Exception {
        runSearch(false);
    }

    @Test(timeout = 150000)
    public void tankSearchRecoversKnownInertiaOnBothAxes() throws Exception {
        runSearch(true);
    }

    private void runSearch(boolean tank) throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                Files.createTempDirectory("apex-ka-phase-").toString());
        SimLinearOpModeBridge.Session session = null;
        try {
            ApexSimulation.Hardware hardware = tank ? ApexSimulation.createTankHardware()
                    : ApexSimulation.createHardware();
            java.util.concurrent.atomic.AtomicReference<TunerContext> captured =
                    new java.util.concurrent.atomic.AtomicReference<>();
            List<String> frames = Collections.synchronizedList(new ArrayList<>());
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            LinearOpMode opMode = new LinearOpMode() {
                @Override public void runOpMode() throws InterruptedException {
                    TunerContext context = new TunerContext(this);
                    context.setFollower(new Follower(tank ? new TankSimulationConstants()
                            : new Constants(), hardwareMap, true));
                    context.constants.translationalFeedforwardKS = .15;
                    context.constants.translationalKV = .01;
                    context.constants.angularFeedforwardKS = .15;
                    context.constants.angularKV = .08;
                    context.constants.forwardVelLimitIn = 65;
                    context.constants.forwardAccelLimitIn = 148;
                    context.constants.angularVelLimitRad = 7;
                    context.constants.angularAccelLimitRad = 14.5;
                    if (tank) {
                        // Representative results of the preceding slow kS/kV ramp.
                        context.constants.translationalFeedforwardKS = .072;
                        context.constants.translationalKV = .01256;
                        context.constants.angularFeedforwardKS = .072;
                        context.constants.angularKV = .088;
                    }
                    captured.set(context);
                    waitForStart();
                    new AccelerationFeedforwardPhase(context).run(this);
                }
            };
            opMode.hardwareMap = hardware.hardwareMap;
            opMode.telemetry = telemetry;
            opMode.gamepad1 = new Gamepad();
            opMode.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(opMode, () -> { });
            SimLinearOpModeBridge.start(session);

            long deadline = System.nanoTime() + 135_000_000_000L;
            long previous = System.nanoTime();
            long pressStart = -1L;
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
                if (frame.contains("phase complete with results")
                        || frame.contains("Automatic kA tuning stopped.")) { break; }
                if (pressStart < 0L && frame.contains("Press A to run this phase")) {
                    pressStart = now;
                }
                opMode.gamepad1.a = pressStart >= 0L && now - pressStart < 60_000_000L;
                SimLinearOpModeBridge.eventLoopIteration(
                        session, opMode.gamepad1, opMode.gamepad2);
                Thread.sleep(5);
            }
            assertTrue("Automatic kA phase did not finish:\n" + frame,
                    frame.contains("phase complete with results")
                            && frame.contains("passed paired-direction confirmation"));
            System.out.println(frame);
            if (tank) {
                org.junit.Assert.assertEquals("Translation inertia", .94 / 150,
                        captured.get().constants.translationalKA, .0005);
                org.junit.Assert.assertEquals("Angular inertia", 7 * .94 / 150,
                        captured.get().constants.angularKA, .010);
                try (java.util.stream.Stream<java.nio.file.Path> files = Files.list(
                        java.nio.file.Paths.get(System.getProperty(ApexStorage.DIRECTORY_PROPERTY)))) {
                    java.nio.file.Path log = files.filter(p -> p.getFileName().toString()
                            .startsWith("acceleration_feedforward_")).findFirst().get();
                    int scoredRows = 0, brakingRows = 0;
                    for (String row : Files.readAllLines(log)) {
                        String[] columns = row.split(",", -1);
                        if (!(columns[2].equals("POSITIVE") || columns[2].equals("NEGATIVE"))
                                || !columns[10].equals("true")) { continue; }
                        scoredRows++;
                        double time = Double.parseDouble(columns[3]);
                        double target = Double.parseDouble(columns[5]);
                        double acceleration = Double.parseDouble(columns[6]);
                        if (acceleration > 0) {
                            org.junit.Assert.assertEquals("Reference must match measurement timestamp",
                                    time * acceleration, target, 1e-9);
                        } else { brakingRows++; }
                    }
                    assertTrue("Both axes should produce scored evidence", scoredRows >= 640);
                    assertTrue("Braking must influence the fitted inertia", brakingRows >= 320);
                    System.out.println("TUNED TRANSLATION KA=" + captured.get().constants.translationalKA);
                    System.out.println("TUNED ANGULAR KA=" + captured.get().constants.angularKA);
                    System.out.println("KA CSV=" + log);
                }
            }
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            if (previousDirectory == null) {
                System.clearProperty(ApexStorage.DIRECTORY_PROPERTY);
            } else {
                System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory);
            }
        }
    }

    private static void step(ApexSimulation.Hardware hardware, double dt) throws Exception {
        double[] wheels = new double[hardware.drivetrain.motorNames.length];
        for (int i = 0; i < wheels.length; i++) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(
                    DcMotorEx.class, hardware.drivetrain.motorNames[i]);
            motor.update(dt);
            wheels[i] = motor.getVelocity();
        }
        java.lang.reflect.Method method = hardware.drivetrain.getClass()
                .getDeclaredMethod("forwardKinematics", double[].class);
        method.setAccessible(true);
        MotionVector velocity = (MotionVector) method.invoke(
                hardware.drivetrain, (Object) wheels);
        hardware.drivetrain.velocity = velocity.toFieldFrame(hardware.drivetrain.position.theta);
        hardware.drivetrain.position = hardware.drivetrain.position.step(
                hardware.drivetrain.velocity, dt);
    }
}
