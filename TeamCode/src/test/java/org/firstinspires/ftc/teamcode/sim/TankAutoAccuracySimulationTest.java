package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.firstinspires.ftc.teamcode.apexpathing.AutoTest;
import org.junit.Test;

import java.lang.reflect.Method;
import java.io.File;
import java.nio.file.Files;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import core.FollowerConstants;
import core.ApexStorage;
import paths.movements.Path;

/** Actual tank AutoTest route using a fixed snapshot of the September 12 simulator tuning. */
public class TankAutoAccuracySimulationTest {
    private static final double MAX_TOTAL_MOVEMENT_SECONDS = 12.0;
    private Object priorConstants;
    private String priorStorage, priorTank;
    private java.lang.reflect.Field singleton;

    @org.junit.Before public void preserveConfiguration() throws Exception {
        priorStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        priorTank = System.getProperty(ApexSimulation.TANK_SIMULATION_PROPERTY);
        singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        priorConstants = singleton.get(null);
        singleton.set(null, null);
    }

    @org.junit.After public void restoreConfiguration() throws Exception {
        singleton.set(null, priorConstants);
        restoreProperty(ApexStorage.DIRECTORY_PROPERTY, priorStorage);
        restoreProperty(ApexSimulation.TANK_SIMULATION_PROPERTY, priorTank);
    }

    private static void restoreProperty(String name, String value) {
        if (value == null) { System.clearProperty(name); }
        else { System.setProperty(name, value); }
    }

    @Test(timeout = 130_000L)
    public void autoTestCompletesEveryMovement() throws Exception {
        runAutoTest();
    }

    @Test(timeout = 130_000L) public void repeatedAutoTestRemainsAccurate() throws Exception {
        runAutoTest();
    }

    @Test(timeout = 130_000L) public void tunedAutoTestBrakesAndSettles() throws Exception {
        try (java.io.InputStream input = getClass().getResourceAsStream("/tank-tuned-constants.json")) {
            java.io.ByteArrayOutputStream bytes = new java.io.ByteArrayOutputStream();
            byte[] buffer = new byte[4096];
            int length;
            while ((length = input.read(buffer)) >= 0) { bytes.write(buffer, 0, length); }
            String tuning = new String(bytes.toByteArray(), java.nio.charset.StandardCharsets.UTF_8);
            runAutoTest(tuning);
            runAutoTest(tuning);
        }
    }

    void runAutoTest() throws Exception {
        runAutoTest(null);
    }

    void runAutoTest(String tuningJson) throws Exception {
        configureTankConstants(tuningJson);
        System.setProperty(ApexSimulation.TANK_SIMULATION_PROPERTY, "true");
        ApexSimulation.Hardware hardware = ApexSimulation.createTankHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        AutoTest auto = new TankAutoTest();
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
            validateAccuracy(new File(auto.getOutboundVelocityCsvPath()));
            if (tuningJson != null) { validateBraking(new File(auto.getOutboundVelocityCsvPath())); }
            System.out.println("AUTO TEST TIMING: " + auto.getMovementTimingReport());
            System.out.println("AUTO TEST EXTERNAL RUNTIME: " + routeRunSeconds + " s");
            assertEquals("Outbound, turn and reverse must all pass", 3, auto.getPassedStages());
            System.out.println("TANK CSV: " + auto.getOutboundVelocityCsvPath());
            System.out.println("TANK RESULT: " + frame);
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
        while (System.nanoTime() < deadline) {
            long now = System.nanoTime();
            double remaining = Math.max(0, Math.min(0.05,
                    (now - hardware.lastPhysicsUpdateNanos) * 1e-9));
            hardware.lastPhysicsUpdateNanos = now;
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

    static void stepPhysics(ApexSimulation.Hardware hardware, double dt) throws Exception {
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

    private static void validateAccuracy(File csv) throws Exception {
        List<String> lines = Files.readAllLines(csv.toPath());
        String[] headers = lines.get(0).split(",");
        int crossColumn = java.util.Arrays.asList(headers).indexOf("cross_track_error_in");
        double peak = 0, squares = 0;
        for (int i = 1; i < lines.size(); i++) {
            double error = Double.parseDouble(lines.get(i).split(",")[crossColumn]);
            assertTrue("Cross-track measurement must be finite", Double.isFinite(error));
            peak = Math.max(peak, Math.abs(error));
            squares += error * error;
        }
        double rms = Math.sqrt(squares / (lines.size() - 1));
        System.out.println("TANK ACCURACY: peak=" + peak + " in, RMS=" + rms + " in");
        assertTrue("Expected a full outbound trace", lines.size() > 100);
        assertTrue("Tank cross-track diagnostic must measure actual drift", peak > 0.1);
        assertTrue("Peak cross-track error: " + peak, peak < 2.2);
        assertTrue("RMS cross-track error: " + rms, rms < 1.0);
    }

    private static void validateBraking(File csv) throws Exception {
        List<String> lines = Files.readAllLines(csv.toPath());
        List<String> headers = java.util.Arrays.asList(lines.get(0).split(","));
        int targetIndex = headers.indexOf("controller_target_in_s");
        int actualIndex = headers.indexOf("controller_measured_in_s");
        int accelerationIndex = headers.indexOf("target_accel_in_s2");
        int blendIndex = headers.indexOf("endpoint_blend");
        double[] squares = new double[3];
        int[] counts = new int[3];
        for (int i = 1; i < lines.size(); i++) {
            String[] row = lines.get(i).split(",");
            double target = Double.parseDouble(row[targetIndex]);
            double actual = Double.parseDouble(row[actualIndex]);
            double acceleration = Double.parseDouble(row[accelerationIndex]);
            double blend = Double.parseDouble(row[blendIndex]);
            if (target <= 6 || Math.abs(acceleration) <= 5) { continue; }
            int region = blend > 0 ? 2 : acceleration < 0 ? 1 : 0;
            double error = actual-target;
            squares[region] += error*error; counts[region]++;
        }
        double[] limits = {1.5, 1.8, 2.0};
        for (int i = 0; i < counts.length; i++) {
            assertTrue("Need acceleration, interior braking and endpoint evidence", counts[i] >= 5);
            double rms = Math.sqrt(squares[i]/counts[i]);
            System.out.println("TANK SPEED REGION " + i + " RMSE=" + rms);
            assertTrue("Speed tracking RMS in region " + i + ": " + rms, rms < limits[i]);
        }
        double finishSpeed = Double.parseDouble(lines.get(lines.size()-1).split(",")[actualIndex]);
        System.out.println("TANK FINISH SPEED=" + finishSpeed);
        assertTrue("AutoTest must actually slow before completing: " + finishSpeed,
                Math.abs(finishSpeed) <= 8.01);
    }

    private static void configureTankConstants(String tuningJson) throws Exception {
        File directory = Files.createTempDirectory("tank-accuracy-").toFile();
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        if (tuningJson != null) {
            Files.write(new File(directory, "constants.json").toPath(),
                    tuningJson.getBytes(java.nio.charset.StandardCharsets.UTF_8));
        } else try (java.io.InputStream input = TankAutoAccuracySimulationTest.class
                .getResourceAsStream("/tank-accuracy-constants.json")) {
            if (input == null) { throw new IllegalStateException("Missing tank tuning fixture"); }
            Files.copy(input, new File(directory, "constants.json").toPath());
        }
        FollowerConstants.getInstance().reload();
    }
}
