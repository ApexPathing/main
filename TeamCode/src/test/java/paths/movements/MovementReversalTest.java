package paths.movements;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;

import java.util.concurrent.atomic.AtomicInteger;

import geometry.Angle;
import geometry.Dist;
import geometry.GeometryFactory;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import org.junit.Test;
import paths.constraint.PathConstraint;
import paths.constraint.TranslationalConstraint;
import paths.heading.InterpolationStyle;

public class MovementReversalTest {
    private static final double EPSILON = 1e-6;

    @Test
    public void reversedPathMirrorsGeometryHeadingAndConstraintSchedule() {
        GeometryFactory factory = new GeometryFactory();
        Path original = factory.holonomicPath(
                        factory.pose(0.0, 0.0, 10.0),
                        factory.pose(18.0, 8.0, 45.0),
                        factory.pose(30.0, 24.0, 100.0))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END)
                .addConstraint(new TranslationalConstraint(0.0,
                        PathConstraint.Type.VELOCITY, Dist.fromIn(24.0)))
                .addConstraint(new TranslationalConstraint(0.5,
                        PathConstraint.Type.VELOCITY, Dist.fromIn(12.0)))
                .quickBuild();

        Path reversed = original.reversed();
        PathSegment source = original.getParametricPath();
        PathSegment reverse = reversed.getParametricPath();

        for (double t : new double[] {0.0, 0.2, 0.5, 0.8, 1.0}) {
            assertVectorEquals(source.getPosition(1.0 - t), reverse.getPosition(t));
            assertVectorEquals(source.getFirstDerivative(1.0 - t).times(-1.0),
                    reverse.getFirstDerivative(t));
            assertEquals(-source.getSignedCurvature(1.0 - t),
                    reverse.getSignedCurvature(t), EPSILON);
        }

        double reverseRemaining = 0.35 * reverse.getLengthIn();
        double sourceRemaining = source.getLengthIn() - reverseRemaining;
        Vector reverseTangent = reverse.getFirstDerivative(0.65);
        Vector sourceTangent = source.getFirstDerivative(0.35);
        Angle expectedHeading = original.getInterpolator().getHeadingTarg(sourceRemaining,
                sourceTangent, source.getFirstDerivative(1.0));
        Angle actualHeading = reversed.getInterpolator().getHeadingTarg(reverseRemaining,
                reverseTangent, reverse.getFirstDerivative(1.0));
        assertEquals(0.0, expectedHeading.getShortestAngleTo(actualHeading).getRad(), EPSILON);

        assertEquals(12.0, reversed.getQuickVelocityLimit(0.25, 40.0), EPSILON);
        assertEquals(24.0, reversed.getQuickVelocityLimit(0.75, 40.0), EPSILON);
        assertEquals(Path.BuildMode.QUICK, reversed.getBuildMode());
    }

    @Test
    public void reversedMovementsClearCallbacksAndExposeTypedFluentMethods() {
        GeometryFactory factory = new GeometryFactory();
        AtomicInteger callbackCount = new AtomicInteger();
        Path original = factory.holonomicPath(factory.pose(0.0, 0.0, 0.0),
                        factory.pose(12.0, 0.0, 0.0))
                .addDistanceCallback(0.5, callbackCount::incrementAndGet)
                .quickBuild();

        Path reversed = original.reversed();
        assertEquals(1, original.getCallbacks().length);
        assertEquals(0, reversed.getCallbacks().length);
        assertSame(reversed, reversed.addDistanceCallback(0.25, callbackCount::incrementAndGet));
        assertSame(reversed, reversed.addAngularCallback(
                Angle.fromRad(0.0), callbackCount::incrementAndGet));
        assertEquals(2, reversed.getCallbacks().length);

        Turn turn = new Turn(factory.pose(4.0, 3.0, 15.0), Angle.fromDeg(80.0));
        turn.addAngularCallback(Angle.fromDeg(45.0), callbackCount::incrementAndGet);
        Turn reversedTurn = turn.reversed();
        assertEquals(0, reversedTurn.getCallbacks().length);
        assertSame(reversedTurn, reversedTurn.addAngularCallback(
                Angle.fromDeg(45.0), callbackCount::incrementAndGet));
    }

    @Test
    public void doubleReversalRestoresMotionButNotCallbacks() {
        GeometryFactory factory = new GeometryFactory();
        Path original = factory.holonomicPath(factory.pose(0.0, 0.0, -20.0),
                        factory.pose(10.0, 14.0, 70.0))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END)
                .addDistanceCallback(0.4, () -> { })
                .quickBuild();

        Path restored = original.reversed().reversed();
        assertNotSame(original, restored);
        assertEquals(0, restored.getCallbacks().length);
        for (double t : new double[] {0.0, 0.25, 0.75, 1.0}) {
            assertVectorEquals(original.getParametricPath().getPosition(t),
                    restored.getParametricPath().getPosition(t));
        }
        assertEquals(0.0, original.getEndPose().getHeading().getShortestAngleTo(
                restored.getEndPose().getHeading()).getRad(), EPSILON);
    }

    private static void assertVectorEquals(Vector expected, Vector actual) {
        assertEquals(0.0, expected.distanceTo(actual).getIn(), EPSILON);
    }
}
