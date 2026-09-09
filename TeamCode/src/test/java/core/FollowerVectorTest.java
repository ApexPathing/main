package core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import controllers.PDSController;

import geometry.DistUnit;
import geometry.Angle;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.movements.Path;
import paths.movements.Turn;

public class FollowerVectorTest {
    @Test
    public void matchingMovingHeadingDoesNotCommandBrakingInEitherDirection() {
        PDSController controller = new PDSController(
                new PDSController.PDSCoefficients(4.1275, 0.7685, 0.24));
        controller.setAngularController();
        for (double rate : new double[] {-2.0, 2.0}) {
            assertEquals("Zero heading error and matching turn rate need no correction",
                    0.0, Follower.calculatePathHeadingFeedback(
                            controller, 0.0, rate, rate, false), 1e-12);
        }
    }

    @Test
    public void stationaryHeadingStillDampsUnwantedRotation() {
        PDSController controller = new PDSController(
                new PDSController.PDSCoefficients(4.0, 0.75, 0.0));
        assertEquals(-0.375, Follower.calculatePathHeadingFeedback(
                controller, 0.0, 0.0, 0.5, false), 1e-12);
    }

    @Test
    public void slowedAndStuckThresholdsAreInclusive() {
        assertFalse(Follower.slowedForVelocities(20.0, 14.000001));
        assertTrue(Follower.slowedForVelocities(20.0, 14.0));
        assertTrue(Follower.slowedForVelocities(20.0, 13.999999));

        assertTrue(Follower.stuckForSpeed(0.499999));
        assertTrue(Follower.stuckForSpeed(0.5));
        assertFalse(Follower.stuckForSpeed(0.500001));
        assertFalse(Follower.stuckForSpeed(Double.NaN));
    }

    @Test
    public void pathStatusExcludesIdlePausedAndTurns() {
        Path path = new Path(Path.PathType.HOLONOMIC);
        Turn turn = new Turn(Pose.zero(), Angle.fromRad(1.0));

        assertTrue(Follower.pathStatusActive(path, false));
        assertFalse(Follower.pathStatusActive(null, false));
        assertFalse(Follower.pathStatusActive(path, true));
        assertFalse(Follower.pathStatusActive(turn, false));
    }

    @Test
    public void stuckWatchdogRequiresOneContinuousHalfSecond() {
        Follower.StuckWatchdog watchdog = new Follower.StuckWatchdog();
        long start = 1_000_000_000L;

        assertFalse(watchdog.update(true, start));
        assertFalse(watchdog.update(true, start + 499_999_999L));
        assertTrue(watchdog.update(true, start + 500_000_000L));

        assertFalse(watchdog.update(false, start + 600_000_000L));
        assertFalse(watchdog.update(true, start + 700_000_000L));
        assertFalse(watchdog.update(true, start + 1_100_000_000L));
        assertTrue(watchdog.update(true, start + 1_200_000_000L));
    }

    @Test
    public void pathCompletionAcceptsInclusiveMinimumBackedBoundaries() {
        assertTrue(Follower.pathInsideCompletionTolerance(
                1.5, Math.toRadians(2.0), 0.5, Math.toRadians(1.0)));
        assertFalse(Follower.pathInsideCompletionTolerance(
                1.500001, Math.toRadians(2.0), 0.5, Math.toRadians(1.0)));
        assertFalse(Follower.pathInsideCompletionTolerance(
                1.5, Math.toRadians(2.000001), 0.5, Math.toRadians(1.0)));

        assertTrue(Follower.pathVelocitySettled(64.0, 0.25));
        assertFalse(Follower.pathVelocitySettled(64.000001, 0.25));
        assertFalse(Follower.pathVelocitySettled(64.0, 0.250001));
    }

    @Test
    public void feedbackNormalizationScalesTranslationAndHeadingTogether() {
        assertEquals(1.5, Follower.commandNormalizationScale(0.8, -0.4, 0.3, true), 1e-9);
        assertEquals(1.0, Follower.commandNormalizationScale(0.3, 0.4, 0.2, false), 1e-9);
        assertEquals(1.3, Follower.commandNormalizationScale(0.3, 0.4, 0.8, false), 1e-9);
    }

    @Test
    public void centripetalVectorPointsInsideForLeftAndRightCurves() {
        Vector tangent = Vector.of(1.0, 0.0, DistUnit.IN);

        Vector leftNormal = PathSegment.calculateArcNormal(
                tangent, Vector.of(0.0, 1.0, DistUnit.IN));
        Vector leftCorrection = Follower.calculateCentripetalCorrection(
                leftNormal, 20.0, 0.05, 0.01);
        assertEquals(0.0, leftCorrection.getX().getIn(), 1e-9);
        assertTrue(leftCorrection.getY().getIn() > 0.0);

        Vector rightNormal = PathSegment.calculateArcNormal(
                tangent, Vector.of(0.0, -1.0, DistUnit.IN));
        Vector rightCorrection = Follower.calculateCentripetalCorrection(
                rightNormal, 20.0, -0.05, 0.01);
        assertEquals(0.0, rightCorrection.getX().getIn(), 1e-9);
        assertTrue(rightCorrection.getY().getIn() < 0.0);
    }

    @Test
    public void centripetalMagnitudeDoesNotDependOnTurnDirection() {
        Vector left = Follower.calculateCentripetalCorrection(
                Vector.of(0.0, 1.0, DistUnit.IN), 30.0, 0.04, 0.002);
        Vector right = Follower.calculateCentripetalCorrection(
                Vector.of(0.0, -1.0, DistUnit.IN), 30.0, -0.04, 0.002);

        assertEquals(left.getMag().getIn(), right.getMag().getIn(), 1e-9);
        assertEquals(-left.getY().getIn(), right.getY().getIn(), 1e-9);
    }

    @Test
    public void crossTrackNormalExistsOnStraightSections() {
        Vector normal = PathSegment.calculateLeftNormal(
                Vector.of(4.0, 0.0, DistUnit.IN));

        assertEquals(0.0, normal.getX().getIn(), 1e-9);
        assertEquals(1.0, normal.getY().getIn(), 1e-9);
    }

    @Test
    public void crossTrackCorrectionDirectionIsIndependentOfWhichSideRobotIsOn() {
        Vector closestPoint = Vector.of(10.0, 0.0, DistUnit.IN);
        Vector leftNormal = PathSegment.calculateLeftNormal(
                Vector.of(1.0, 0.0, DistUnit.IN));

        Vector robotOnRight = Vector.of(10.0, -3.0, DistUnit.IN);
        double rightError = closestPoint.minus(robotOnRight).dot(leftNormal).getIn();
        Vector rightCorrection = leftNormal.times(rightError);
        assertTrue(rightCorrection.getY().getIn() > 0.0);

        Vector robotOnLeft = Vector.of(10.0, 3.0, DistUnit.IN);
        double leftError = closestPoint.minus(robotOnLeft).dot(leftNormal).getIn();
        Vector leftCorrection = leftNormal.times(leftError);
        assertTrue(leftCorrection.getY().getIn() < 0.0);
    }

    @Test
    public void profiledFeedforwardUsesAccelerationSignWhenStartingFromRest() {
        assertEquals(1.0, Follower.feedforwardMotionSign(0.0, 30.0), 1e-9);
        assertEquals(-1.0, Follower.feedforwardMotionSign(0.0, -30.0), 1e-9);
        assertEquals(1.0, Follower.feedforwardMotionSign(10.0, -30.0), 1e-9);
        assertEquals(0.0, Follower.feedforwardMotionSign(0.0, 0.0), 1e-9);
    }

    @Test
    public void headingFeedforwardUsesLocalTrajectoryTimeScaling() {
        assertEquals(1.0, Follower.headingFeedforwardTimeScale(40.0, 40.0), 1e-9);
        assertEquals(0.5, Follower.headingFeedforwardTimeScale(40.0, 20.0), 1e-9);
        assertEquals(0.0, Follower.headingFeedforwardTimeScale(40.0, -2.0), 1e-9);
        assertEquals(1.0, Follower.headingFeedforwardTimeScale(40.0, 50.0), 1e-9);
        assertEquals(0.0, Follower.headingFeedforwardTimeScale(0.0, 10.0), 1e-9);
    }

    @Test
    public void brakingFeedforwardOpposesMotionInEitherDirection() {
        assertEquals(-0.36, Follower.calculateTranslationFeedforward(
                30.0, -100.0, 0.008, 0.007, 0.10), 1e-9);
        assertEquals(0.36, Follower.calculateTranslationFeedforward(
                -30.0, 100.0, 0.008, 0.007, 0.10), 1e-9);
        assertEquals(1.04, Follower.calculateTranslationFeedforward(
                30.0, 100.0, 0.008, 0.007, 0.10), 1e-9);
    }

    @Test
    public void spatialBrakingReleasesAtRestAndRespectsTravelDirection() {
        assertEquals(0.0, Follower.scaleBrakingAcceleration(30, -100, 0), 1e-12);
        assertEquals(-25.0, Follower.scaleBrakingAcceleration(30, -100, 15), 1e-12);
        assertEquals(25.0, Follower.scaleBrakingAcceleration(-30, 100, -15), 1e-12);
        assertEquals(-100.0, Follower.scaleBrakingAcceleration(30, -100, 40), 1e-12);
        assertEquals(0.0, Follower.scaleBrakingAcceleration(30, -100, -5), 1e-12);
        assertEquals(100.0, Follower.scaleBrakingAcceleration(30, 100, 0), 1e-12);
        assertEquals(100.0, Follower.scaleBrakingAcceleration(0, 100, 0), 1e-12);
    }

    @Test
    public void profiledEndpointCaptureSuppliesPowerAfterVelocityProfileStops() {
        assertEquals(0.0, Follower.blendProfiledEndpointPower(0.0, 0.25, 8.0), 1e-9);
        assertEquals(0.125, Follower.blendProfiledEndpointPower(0.0, 0.25, 4.0), 1e-9);
        assertEquals(0.25, Follower.blendProfiledEndpointPower(0.0, 0.25, 0.0), 1e-9);
    }

    @Test
    public void endpointTangentErrorKeepsCorrectSignOnBothTravelDirections() {
        Vector endpoint = Vector.of(-24.0, 0.0, DistUnit.IN);
        Vector current = Vector.of(-22.0, 0.0, DistUnit.IN);
        Vector returnTangent = Vector.of(-1.0, 0.0, DistUnit.IN);

        assertEquals(2.0,
                Follower.pathEndpointTangentError(endpoint, current, returnTangent), 1e-9);
    }

    @Test
    public void stalledEndpointCommandClearsStaticFrictionDeadband() {
        assertEquals(0.42375, Follower.ensureEndpointBreakawayPower(
                0.18, 0.60, 0.0, 0.24375, 0.50, 0.0), 1e-9);
        assertEquals(-0.42375, Follower.ensureEndpointBreakawayPower(
                -0.18, -0.60, 0.0, 0.24375, 0.50, 0.0), 1e-9);

        // Never inject breakaway power after reaching tolerance or while already moving.
        assertEquals(0.18, Follower.ensureEndpointBreakawayPower(
                0.18, 0.40, 0.0, 0.24375, 0.50, 0.0), 1e-9);
        assertEquals(0.18, Follower.ensureEndpointBreakawayPower(
                0.18, 0.60, 1.0, 0.24375, 0.50, 0.0), 1e-9);
    }

    @Test
    public void stalledCrossTrackEndpointCommandAddsStaticFriction() {
        assertEquals(0.37125, Follower.ensureEndpointBreakawayPower(
                0.14, 1.14, 0.0, 0.23125,
                controllers.PDSController.LINEAR_STATIC_DEADBAND, 0.04), 1e-9);

        // Inside the PDS deadband, no breakaway command should be injected.
        assertEquals(0.02, Follower.ensureEndpointBreakawayPower(
                0.02, 0.20, 0.0, 0.23125,
                controllers.PDSController.LINEAR_STATIC_DEADBAND, 0.04), 1e-9);
    }

    @Test
    public void stalledProfiledTurnClearsStaticFrictionDeadband() {
        assertEquals(0.24375, Follower.ensureAngularEndpointBreakawayPower(
                0.0, Math.toRadians(3.25), 0.0, 0.24375, Math.toRadians(1.0)), 1e-9);
        assertEquals(-0.24375, Follower.ensureAngularEndpointBreakawayPower(
                0.0, Math.toRadians(-3.25), 0.0, 0.24375, Math.toRadians(1.0)), 1e-9);
        assertEquals(0.0, Follower.ensureAngularEndpointBreakawayPower(
                0.0, Math.toRadians(0.5), 0.0, 0.24375, Math.toRadians(1.0)), 1e-9);
    }
}
