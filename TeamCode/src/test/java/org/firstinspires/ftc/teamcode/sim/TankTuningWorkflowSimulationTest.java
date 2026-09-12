package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import core.*;
import org.junit.Test;
import java.nio.file.Files;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import tuning.follower.TunerContext;
import tuning.follower.phases.*;
import static org.junit.Assert.*;

/** Runs actual tuner state machines in sequence, then validates their output on the tank route. */
public class TankTuningWorkflowSimulationTest {
    @Test(timeout = 600000) public void measuredTuningImprovesTankTracking() throws Exception {
        runWorkflow(false);
    }

    @Test(timeout = 180000) public void retunedFeedbackTracksConsistentProfile() throws Exception {
        runWorkflow(true);
    }

    private void runWorkflow(boolean feedbackOnly) throws Exception {
        TankAutoAccuracySimulationTest route = new TankAutoAccuracySimulationTest();
        route.preserveConfiguration();
        SimLinearOpModeBridge.Session session = null;
        try {
            java.nio.file.Path directory = Files.createTempDirectory("tank-tuning-workflow-");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.toString());
            System.setProperty(ApexSimulation.TANK_SIMULATION_PROPERTY, "true");
            try (java.io.InputStream input = getClass().getResourceAsStream(feedbackOnly
                    ? "/tank-tuned-constants.json" : "/tank-accuracy-constants.json")) {
                Files.copy(input, directory.resolve("constants.json"));
            }
            FollowerConstants.getInstance().reload();
            ApexSimulation.Hardware hardware = ApexSimulation.createTankHardware();
            List<String> frames = new CopyOnWriteArrayList<>();
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            java.util.concurrent.atomic.AtomicReference<String> output = new java.util.concurrent.atomic.AtomicReference<>();
            LinearOpMode op = new LinearOpMode() {
                @Override public void runOpMode() {
                    TunerContext context = new TunerContext(this);
                    context.setFollower(new Follower(new TankSimulationConstants(), hardwareMap, true));
                    waitForStart();
                    if (!feedbackOnly) {
                        if (!new FeedforwardTuner(context).run(this)) { return; }
                        System.out.println("AFTER RAMP " + context.constants.toJson());
                        if (!new AccelerationFeedforwardPhase(context).run(this)) { return; }
                        System.out.println("AFTER KA " + context.constants.toJson());
                    }
                    if (!new VelocityFeedbackPhase(context).run(this)) { return; }
                    output.set(context.constants.toJson().toString());
                    context.getFollower().stop();
                }
            };
            op.hardwareMap = hardware.hardwareMap;
            op.telemetry = telemetry;
            op.gamepad1 = new Gamepad(); op.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(op, () -> {});
            SimLinearOpModeBridge.start(session);
            long deadline = System.nanoTime() + 550_000_000_000L;
            long previous = System.nanoTime(), lastPress = 0;
            String frame = "";
            while (output.get() == null && System.nanoTime() < deadline) {
                long now = System.nanoTime();
                double remaining = Math.min(.05, (now-previous)*1e-9); previous = now;
                while (remaining > 1e-9) {
                    double dt = Math.min(.005, remaining);
                    TankAutoAccuracySimulationTest.stepPhysics(hardware, dt); remaining -= dt;
                }
                frame = frames.isEmpty() ? "" : frames.get(frames.size()-1);
                if (frame.contains("Automatic kA tuning stopped.")) { break; }
                if (frame.contains("Press A") && now-lastPress > 300_000_000L) { lastPress = now; }
                op.gamepad1.a = now-lastPress < 60_000_000L;
                SimLinearOpModeBridge.eventLoopIteration(session, op.gamepad1, op.gamepad2);
                Thread.sleep(5);
            }
            assertNotNull("Tuner workflow did not finish: " + frame, output.get());
            SimLinearOpModeBridge.stop(session); session = null;
            Files.write(directory.resolve("validated-tuning.json"), output.get().getBytes(java.nio.charset.StandardCharsets.UTF_8));
            System.out.println("WORKFLOW TUNING " + directory.resolve("validated-tuning.json"));
            System.out.println("WORKFLOW VALUES " + output.get());
            route.runAutoTest(output.get());
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            route.restoreConfiguration();
        }
    }
}
