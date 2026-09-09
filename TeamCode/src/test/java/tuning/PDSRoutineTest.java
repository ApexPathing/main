package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PDSRoutineTest {
    @Test
    public void linearPositionIsRelativeToTrialStart() {
        assertEquals(2.5, PDSRoutine.relativePosition(12.5, 10.0, false), 1e-9);
    }

    @Test
    public void angularPositionWrapsAcrossPiBoundary() {
        double start = Math.toRadians(179.0);
        double current = Math.toRadians(-179.0);

        assertEquals(Math.toRadians(2.0),
                PDSRoutine.relativePosition(current, start, true), 1e-9);
    }

    @Test
    public void trialTargetsStayAnchoredDespiteStoppingError() {
        assertEquals(24.0, PDSRoutine.anchoredTarget(1.0, 24.0), 0.0);
        assertEquals(0.0, PDSRoutine.anchoredTarget(-1.0, 24.0), 0.0);
    }

    @Test
    public void configRejectsUnsafeInputs() {
        assertThrows(IllegalArgumentException.class,
                () -> PDSRoutine.Config.linear(
                        "drive", 0.75, 0.01, 0.0, 0.3,
                        24.0, 0.75, 1.0, 36.0));
        assertThrows(IllegalArgumentException.class,
                () -> PDSRoutine.Config.linear(
                        "drive", Double.NaN, 0.75, 0.0, 0.3,
                        24.0, 0.75, 1.0, 36.0));
    }

    @Test
    public void routineRejectsInvalidGenericGuesses() {
        PDSRoutine.Config config = PDSRoutine.Config.linear(
                "drive", 0.01, 0.75, 0.0, 0.3,
                24.0, 0.75, 1.0, 36.0);

        assertThrows(IllegalArgumentException.class,
                () -> new PDSRoutine(config, Double.NaN, 0.01, 0.2));
        assertThrows(IllegalArgumentException.class,
                () -> new PDSRoutine(config, 0.01, -0.01, 0.2));
    }

    @Test
    public void routineStartsFromProvidedGenericGuess() {
        PDSRoutine.Config config = PDSRoutine.Config.angular(
                "heading", 0.10, 32.0, 0.0, 2.0,
                Math.toRadians(60.0), Math.toRadians(1.0), 0.10,
                Math.toRadians(110.0));
        PDSRoutine routine = new PDSRoutine(config, 0.80, 0.10, 0.23);

        routine.start();

        assertEquals(0.80, routine.getCoefficients().kP, 1e-9);
        assertEquals(0.10, routine.getCoefficients().kD, 1e-9);
        assertEquals(0.23, routine.getCoefficients().kS, 1e-9);
    }

    @Test
    public void completedTuneRequiresExplicitUserAcceptance() {
        PDSRoutine.Config config = PDSRoutine.Config.linear(
                "drive", 0.01, 0.75, 0.0, 0.3,
                24.0, 0.75, 1.0, 36.0);
        PDSRoutine routine = new PDSRoutine(config, 0.02, 0.004, 0.2);

        routine.start();

        assertTrue(!routine.isComplete());
    }
}
