package tuning;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import geometry.GeometryFactory;
import geometry.PathPoint;
import geometry.Pose;
import org.junit.Test;
import paths.movements.Path;

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
    public void errorSignSelectsGainDirectionWithoutDeadband() {
        assertEquals(BinarySearch.SearchDirection.HIGHER,
                CentripetalPhase.searchDirection(0.000001));
        assertEquals(BinarySearch.SearchDirection.LOWER,
                CentripetalPhase.searchDirection(0.0));
        assertEquals(BinarySearch.SearchDirection.LOWER,
                CentripetalPhase.searchDirection(-0.5));
    }

    @Test
    public void testArcHasStableThirtyTwoInchRadius() {
        GeometryFactory factory = new GeometryFactory();
        Pose[] points = CentripetalPhase.createTestArc(factory);
        Path path = factory.path(points).quickBuild();

        double minimumCurvature = Double.POSITIVE_INFINITY;
        double maximumCurvature = 0.0;
        for (PathPoint point : path.getGeneratedPoints()) {
            if (point.getT() <= 0.25 || point.getT() >= 0.75) { continue; }
            double curvature = Math.abs(point.getSignedCurvature());
            minimumCurvature = Math.min(minimumCurvature, curvature);
            maximumCurvature = Math.max(maximumCurvature, curvature);
        }

        assertEquals(1.0 / 32.0, minimumCurvature, 0.001);
        assertEquals(1.0 / 32.0, maximumCurvature, 0.001);
    }

    @Test
    public void testVelocityRespectsLateralAndTotalPowerBudgets() {
        GeometryFactory factory = new GeometryFactory();
        Path path = factory.path(CentripetalPhase.createTestArc(factory)).quickBuild();
        double gain = 0.008;
        double kV = 0.01;
        double kS = 0.05;
        double velocity = CentripetalPhase.safeTestVelocity(path, gain, 80.0, kV, kS);

        double maximumCurvature = 0.0;
        for (PathPoint point : path.getGeneratedPoints()) {
            if (point.getT() <= 0.20 || point.getT() >= 0.80) { continue; }
            maximumCurvature = Math.max(maximumCurvature,
                    Math.abs(point.getSignedCurvature()));
        }
        double lateralPower = velocity * velocity * maximumCurvature * gain;
        double totalPower = kS + velocity * kV + lateralPower;

        assertTrue(lateralPower <= 0.20 + 1e-9);
        assertTrue(totalPower <= 0.75 + 1e-9);
        assertTrue(velocity <= 0.75 * 80.0);
    }
}
