package tuning.localizer;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import tuning.localizer.phases.RelativeImuAccumulator;

public class RelativeImuAccumulatorTest {
    @Test public void accumulatesMultipleTurnsAboutArbitraryMountingAxis() {
        RelativeImuAccumulator accumulator = new RelativeImuAccumulator();
        accumulator.resetMeasurement(rotation(0.0));
        for (int i = 1; i <= 80; i++) { accumulator.update(rotation(i * Math.PI / 20.0)); }
        assertEquals(4.0 * Math.PI, accumulator.getAngleRad(), 2e-5);
        assertTrue(accumulator.hasSpinAxis());
        assertEquals(1.0 / Math.sqrt(14.0), accumulator.getSpinAxisX(), 2e-5);
        assertEquals(2.0 / Math.sqrt(14.0), accumulator.getSpinAxisY(), 2e-5);
        assertEquals(3.0 / Math.sqrt(14.0), accumulator.getSpinAxisZ(), 2e-5);
        assertEquals(0.0, accumulator.getRmsOffAxisRad(), 2e-5);

        accumulator.resetMeasurement(rotation(4.0 * Math.PI));
        for (int i = 1; i <= 40; i++) {
            accumulator.update(rotation(4.0 * Math.PI - i * Math.PI / 20.0));
        }
        assertEquals(-2.0 * Math.PI, accumulator.getAngleRad(), 2e-5);
    }

    private static Quaternion rotation(double angle) {
        double length = Math.sqrt(14.0);
        double scale = Math.sin(angle / 2.0) / length;
        return new Quaternion((float) Math.cos(angle / 2.0), (float) scale,
                (float) (2.0 * scale), (float) (3.0 * scale), 0L);
    }
}
