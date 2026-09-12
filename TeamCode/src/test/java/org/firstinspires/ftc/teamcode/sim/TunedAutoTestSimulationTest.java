package org.firstinspires.ftc.teamcode.sim;

import core.ApexStorage;
import org.junit.Test;
import java.io.File;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import static org.junit.Assert.assertTrue;

/** Reproduces the user workflow: every automatic tuning phase, then AutoTest. */
public class TunedAutoTestSimulationTest {
    @Test(timeout = 460_000L)
    public void freshlyTunedConstantsCompleteAutoTest() throws Exception {
        String previous = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File directory = new File("build/tuner-e2e-data");
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        try {
            new FollowerTunerTelemetryTest().automaticWorkflowCompletesFromStaticFrictionThroughFeedback();
            new AutoTestSimulationTest().runAutoTest(true);
            assertVelocityTracking(directory);
        } finally {
            if (previous == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previous); }
        }
    }

    private static void assertVelocityTracking(File directory) throws Exception {
        File[] traces = directory.listFiles((dir, name) -> name.startsWith("auto-test-outbound-velocity_")
                && name.endsWith(".csv"));
        assertTrue("AutoTest must produce a velocity trace", traces != null && traces.length > 0);
        File trace = Arrays.stream(traces).max(Comparator.comparingLong(File::lastModified)).get();
        List<String> lines = Files.readAllLines(trace.toPath());
        String[] headers = lines.get(0).split(",");
        Map<String, Integer> columns = new HashMap<>();
        for (int i = 0; i < headers.length; i++) { columns.put(headers[i], i); }
        double accelerationPeak = 0, brakingSquared = 0;
        int brakingSamples = 0;
        for (int i = 1; i < lines.size(); i++) {
            String[] row = lines.get(i).split(",");
            double target = Double.parseDouble(row[columns.get("controller_target_in_s")]);
            if (target <= 0) { continue; }
            double error = Double.parseDouble(row[columns.get("controller_measured_in_s")]) - target;
            double acceleration = Double.parseDouble(row[columns.get("target_accel_in_s2")]);
            if (acceleration > 5) { accelerationPeak = Math.max(accelerationPeak, error); }
            if (acceleration < -5) { brakingSquared += error * error; brakingSamples++; }
        }
        assertTrue("Freshly tuned acceleration overspeed: " + accelerationPeak, accelerationPeak < 10);
        assertTrue("Expected braking measurements", brakingSamples > 10);
        double brakingRms = Math.sqrt(brakingSquared / brakingSamples);
        assertTrue("Freshly tuned braking RMS: " + brakingRms, brakingRms < 8);
    }
}
