package org.firstinspires.ftc.teamcode.sim;

import core.ApexStorage;
import org.junit.Test;

import java.io.File;
import java.io.InputStream;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

/** Reproduces overshoot with the user's saved simulator tuning, without retuning between runs. */
public class CornerTrackingSimulationTest {
    @Test(timeout = 130_000L)
    public void savedTuningBrakesForAutoTestCornersWithoutLargeTrackingError() throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File directory = Files.createTempDirectory("apex-corner-regression-").toFile();
        try {
            try (InputStream constants = getClass().getResourceAsStream(
                    "/corner-tracking-constants.json")) {
                assertNotNull("Frozen simulator tuning fixture", constants);
                Files.copy(constants, new File(directory, "constants.json").toPath());
            }
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
            new AutoTestSimulationTest().runAutoTest(true);

            File[] traces = directory.listFiles((dir, name) ->
                    name.startsWith("auto-test-outbound-velocity_") && name.endsWith(".csv"));
            assertTrue("Expected an outbound trace", traces != null && traces.length > 0);
            File trace = Arrays.stream(traces).max(Comparator.comparingLong(File::lastModified)).get();
            assertTracking(trace);
        } finally {
            if (previousDirectory == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory); }
        }
    }

    private static void assertTracking(File trace) throws Exception {
        List<String> lines = Files.readAllLines(trace.toPath());
        Map<String, Integer> columns = new HashMap<>();
        String[] header = lines.get(0).split(",");
        for (int i = 0; i < header.length; i++) { columns.put(header[i], i); }
        double peakCrossTrack = 0, peakCurveError = 0, peakHeadingError = 0;
        double brakingSquaredError = 0;
        int curveSamples = 0, brakingSamples = 0;
        for (int i = 1; i < lines.size(); i++) {
            String[] row = lines.get(i).split(",");
            double crossTrack = Math.abs(value(row, columns, "cross_track_error_in"));
            peakCrossTrack = Math.max(peakCrossTrack, crossTrack);
            if (Math.abs(value(row, columns, "curvature_in_inv")) > 0.01) {
                curveSamples++;
                peakCurveError = Math.max(peakCurveError, crossTrack);
                peakHeadingError = Math.max(peakHeadingError,
                        Math.abs(value(row, columns, "heading_error_rad")));
            }
            double target = value(row, columns, "controller_target_in_s");
            if (target > 0 && value(row, columns, "target_accel_in_s2") < -5) {
                double error = value(row, columns, "controller_measured_in_s") - target;
                brakingSquaredError += error * error;
                brakingSamples++;
            }
        }
        assertTrue("Expected corner and braking measurements", curveSamples > 30 && brakingSamples > 30);
        // Include corner exits and the final straight: filtering only curved samples hid drift.
        assertTrue("Outbound cross-track peak: " + peakCrossTrack, peakCrossTrack < 1.25);
        assertTrue("Curve cross-track peak: " + peakCurveError, peakCurveError < 0.75);
        assertTrue("Curve heading peak (deg): " + Math.toDegrees(peakHeadingError),
                peakHeadingError < Math.toRadians(25));
        double brakingRms = Math.sqrt(brakingSquaredError / brakingSamples);
        assertTrue("Braking velocity RMS: " + brakingRms, brakingRms < 8);
        System.out.println("CORNER REGRESSION: outboundPeak=" + peakCrossTrack
                + " curvePeak=" + peakCurveError + " headingPeakDeg="
                + Math.toDegrees(peakHeadingError) + " brakingRms=" + brakingRms);
    }

    private static double value(String[] row, Map<String, Integer> columns, String column) {
        return Double.parseDouble(row[columns.get(column)]);
    }
}
