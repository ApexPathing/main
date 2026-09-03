package tuning;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class CentripetalPhaseTest {
    @Test
    public void turnsAroundNearEndpointWithoutWaitingForFullPoseSettling() {
        assertTrue(CentripetalPhase.readyForTurnaround(0.99, 1.0, 4.0));
    }

    @Test
    public void doesNotTurnAroundEarlyFarAwayOrAtHighSpeed() {
        assertFalse(CentripetalPhase.readyForTurnaround(0.95, 1.0, 4.0));
        assertFalse(CentripetalPhase.readyForTurnaround(0.99, 2.0, 4.0));
        assertFalse(CentripetalPhase.readyForTurnaround(0.99, 1.0, 8.0));
        assertFalse(CentripetalPhase.readyForTurnaround(Double.NaN, 1.0, 4.0));
    }

    @Test
    public void smallErrorSelectsLessAggressiveGain() {
        assertEquals(BinarySearch.SearchDirection.LOWER,
                CentripetalPhase.searchDirection(0.15));
        assertEquals(BinarySearch.SearchDirection.LOWER,
                CentripetalPhase.searchDirection(-0.5));
        assertEquals(BinarySearch.SearchDirection.HIGHER,
                CentripetalPhase.searchDirection(0.16));
    }
}
