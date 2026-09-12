package tuning.follower.phases;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class AccelerationSearchTest {
    @Test public void angularSearchStartsNearMeasuredInertiaInsteadOfPowerCeiling() {
        double upper = AccelerationFeedforwardPhase.initialUpperBound(.072, .088,
                1.2, 1.2, 19.38);
        assertEquals(2 * .928 / 19.38, upper, 1e-12);
        assertTrue(upper < AccelerationFeedforwardPhase.safeUpperBound(.072, .088, 1.2, 1.2));
        assertTrue(Double.isNaN(AccelerationFeedforwardPhase.initialUpperBound(
                .072, .088, 1.2, 1.2, 0)));
    }

    @Test public void confirmationAllowsSmallNoiseWithoutAcceptingRealRegression() {
        assertTrue(AccelerationFeedforwardPhase.confirmationAcceptable(.108, .238, 18));
        assertTrue(AccelerationFeedforwardPhase.confirmationAcceptable(0, .01, 1.2));
        org.junit.Assert.assertFalse(
                AccelerationFeedforwardPhase.confirmationAcceptable(.108, 1, 18));
        org.junit.Assert.assertFalse(
                AccelerationFeedforwardPhase.confirmationAcceptable(.01, .1, 1.2));
        org.junit.Assert.assertFalse(
                AccelerationFeedforwardPhase.confirmationAcceptable(.1, Double.NaN, 18));
    }

    @Test public void signedLagRaisesCandidateAndLeadLowersIt() {
        AccelerationSearch search = new AccelerationSearch(.08, 3);
        assertEquals(.04, search.current(), 1e-12);
        search.record(2, 1);
        assertEquals(.06, search.current(), 1e-12);
        search.record(1, -1);
        assertEquals(.05, search.current(), 1e-12);
        search.record(1.5, 1);
        assertTrue(search.isComplete());
        assertEquals(.06, search.bestCandidate(), 1e-12);
        assertEquals(1, search.bestRms(), 1e-12);
    }

    @Test public void safeUpperBoundLeavesCommandHeadroom() {
        assertEquals(.02, AccelerationFeedforwardPhase.safeUpperBound(
                .15, .02, 20, 15), 1e-12);
        assertTrue(Double.isNaN(AccelerationFeedforwardPhase.safeUpperBound(
                .5, .02, 20, 15)));
    }

    @Test public void pairedScoreUsesWorseDirection() {
        AccelerationFeedforwardPhase.TrialResult result =
                AccelerationFeedforwardPhase.TrialResult.of(
                        new double[] { 4, 9 }, new double[] { 1, -2 },
                        new double[] { 1, 1 }, new int[] { 25, 25 }, false);
        assertTrue(result.valid);
        assertEquals(3, result.rms, 1e-12);
        assertEquals(-.5, result.meanError, 1e-12);
    }
}
