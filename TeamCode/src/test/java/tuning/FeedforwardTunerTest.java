package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class FeedforwardTunerTest {
    @Test
    public void exactDistanceProfileUsesRequestedDriveAndTurnDistances() {
        FeedforwardTuner.TrapezoidProfile drive =
                new FeedforwardTuner.TrapezoidProfile(24.0, 48.0, 48.0);
        FeedforwardTuner.TrapezoidProfile turn =
                new FeedforwardTuner.TrapezoidProfile(4.0, 8.0, Math.PI);

        assertEquals(48.0, drive.getTotalDistance(), 1e-12);
        assertEquals(Math.PI, turn.getTotalDistance(), 1e-12);
        assertEquals(0.0, drive.getVel(drive.getTotalTime()), 1e-12);
        assertEquals(0.0, turn.getVel(turn.getTotalTime()), 1e-12);
    }

    @Test
    public void shortExactDistanceProfileBecomesTriangular() {
        FeedforwardTuner.TrapezoidProfile profile =
                new FeedforwardTuner.TrapezoidProfile(10.0, 2.0, 2.0);

        assertEquals(2.0, profile.getTotalDistance(), 1e-12);
        assertEquals(1.0, profile.getAccelEnd(), 1e-12);
        assertEquals(profile.getAccelEnd(), profile.getCruiseEnd(), 1e-12);
    }

    @Test
    public void holdingBiasCorrectionOpposesMeasuredError() {
        assertEquals(.184, FeedforwardTuner.correctedHoldingKS(.19, .004, 1.5), 1e-12);
        assertEquals(.196, FeedforwardTuner.correctedHoldingKS(.19, .004, -1.5), 1e-12);
        assertEquals(0, FeedforwardTuner.correctedHoldingKS(.01, .004, 10), 1e-12);
    }

    @Test
    public void curvedHoldingDataDoesNotProduceExcessHoldingPower() {
        List<FeedforwardTuner.HoldingPoint> holds = new ArrayList<>();
        List<FeedforwardTuner.IntegralObservation> windows = new ArrayList<>();
        for (int direction = 0; direction < 2; direction++) {
            holds.add(new FeedforwardTuner.HoldingPoint(10, .231));
            holds.add(new FeedforwardTuner.HoldingPoint(23, .271));
            holds.add(new FeedforwardTuner.HoldingPoint(36, .329));
        }
        for (int i = 0; i < 4; i++) {
            windows.add(new FeedforwardTuner.IntegralObservation(.2, 4, 6, .10));
        }
        double[] fit = FeedforwardTuner.fitSeparated(holds, windows);
        assertTrue(FeedforwardTuner.physicalFit(fit));
        for (FeedforwardTuner.HoldingPoint hold : holds) {
            assertTrue(fit[0] + fit[1] * hold.velocity <= hold.power + 1e-10);
        }
    }

    @Test
    public void validationRejectsPersistentOverspeedEvenBelowRmsTolerance() {
        assertTrue(!FeedforwardTuner.validationPhasePassed(25, 25 * 2.3 * 2.3, 25 * 2.3, 3.23));
        assertTrue(!FeedforwardTuner.validationPhasePassed(25, 25 * 2.3 * 2.3, -25 * 2.3, 3.23));
        assertTrue(FeedforwardTuner.validationPhasePassed(25, 25, 0, 3.23));
        assertTrue(!FeedforwardTuner.validationPhasePassed(2, 0, 0, 3.23));
        assertTrue(!FeedforwardTuner.validationPhasePassed(25, Double.NaN, 0, 3.23));
    }

    @Test
    public void separatedFitFixesHoldingTermsBeforeAccelerationAndIgnoresBraking() {
        List<FeedforwardTuner.HoldingPoint> holds = new ArrayList<>();
        List<FeedforwardTuner.IntegralObservation> windows = new ArrayList<>();
        for (int direction = 0; direction < 2; direction++) {
            for (double velocity : new double[]{10, 20, 35}) {
                holds.add(new FeedforwardTuner.HoldingPoint(velocity, .19 + .0042 * velocity));
            }
        }
        for (int i = 1; i <= 20; i++) {
            double distance = .3 * i, deltaVelocity = .5 + i * .1;
            windows.add(new FeedforwardTuner.IntegralObservation(.2, distance, deltaVelocity,
                    .19 * .2 + .0042 * distance + .0073 * deltaVelocity));
        }
        double[] fit = FeedforwardTuner.fitSeparated(holds, windows);
        assertTrue(FeedforwardTuner.physicalFit(fit));
        assertEquals(.19, fit[0], 1e-10);
        assertEquals(.0042, fit[1], 1e-10);
        assertEquals(.0073, fit[2], 1e-10);
    }

    @Test
    public void separatedFitRejectsMissingOrDegenerateEvidence() {
        List<FeedforwardTuner.HoldingPoint> holds = new ArrayList<>();
        List<FeedforwardTuner.IntegralObservation> windows = new ArrayList<>();
        assertTrue(!FeedforwardTuner.physicalFit(FeedforwardTuner.fitSeparated(holds, windows)));
        for (int i = 0; i < 6; i++) {
            holds.add(new FeedforwardTuner.HoldingPoint(10, .25));
        }
        assertTrue(!FeedforwardTuner.physicalFit(FeedforwardTuner.fitSeparated(holds, windows)));
        holds.clear();
        for (int i = 1; i <= 6; i++) {
            holds.add(new FeedforwardTuner.HoldingPoint(i * 5, .19 + .0042 * i * 5));
        }
        assertTrue(!FeedforwardTuner.physicalFit(FeedforwardTuner.fitSeparated(holds, windows)));
    }

    @Test
    public void manualAngularPowerAlternatesDirectionAndClampsSaturation() {
        double counterclockwise = FeedforwardTuner.manualAngularPower(
                0.2, 0.1, 0.15, 3.0, 2.0, 1.0);
        double clockwise = FeedforwardTuner.manualAngularPower(
                0.2, 0.1, 0.15, 3.0, 2.0, -1.0);

        assertEquals(-counterclockwise, clockwise, 1e-12);
        assertEquals(0.95, counterclockwise, 1e-12);
        assertEquals(1.0, FeedforwardTuner.clipManualPower(1.4), 0.0);
        assertEquals(-1.0, FeedforwardTuner.clipManualPower(-1.4), 0.0);
    }

    @Test
    public void kvCruiseRmsLatchesAtFirstBoundedSampleUntilDeceleration() {
        FeedforwardTuner.ManualErrorMetrics metrics =
                new FeedforwardTuner.ManualErrorMetrics();

        assertTrue(!metrics.record(4.0, 4.0, 10.0, true, 1.0));
        assertTrue(metrics.record(10.0, 9.25, 10.0, true, 1.0));
        // Once latched, this remains part of kV RMS even though it leaves the entry bound.
        assertTrue(metrics.record(10.0, 7.0, 10.0, true, 1.0));
        // Deceleration remains part of whole-run kA RMS but never enters cruise kV RMS.
        assertTrue(!metrics.record(8.0, 6.0, 10.0, false, 1.0));

        assertEquals(Math.sqrt((.75 * .75 + 3.0 * 3.0) / 2.0),
                metrics.cruiseRms(), 1e-12);
        assertEquals(Math.sqrt((0.0 + .75 * .75 + 3.0 * 3.0 + 2.0 * 2.0) / 4.0),
                metrics.wholeRunRms(), 1e-12);
    }

    @Test
    public void kvCruiseRmsRemainsEmptyWhenCruiseTargetIsNeverReached() {
        FeedforwardTuner.ManualErrorMetrics metrics =
                new FeedforwardTuner.ManualErrorMetrics();

        assertTrue(!metrics.record(10.0, 8.9, 10.0, true, 1.0));
        assertTrue(!metrics.record(8.0, 8.0, 10.0, false, 1.0));
        assertTrue(Double.isNaN(metrics.cruiseRms()));
        assertEquals(Math.sqrt(1.21 / 2.0), metrics.wholeRunRms(), 1e-12);
    }

}
