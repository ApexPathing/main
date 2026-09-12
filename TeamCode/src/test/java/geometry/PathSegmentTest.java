package geometry;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import paths.builders.HolonomicPathBuilder;

public class PathSegmentTest {
    @Test
    public void cachedCurvatureDerivativeMatchesRuntimeGeometry() {
        PathSegment segment = new PathSegment(new Parabola(1.0));
        assertCachedDerivativesMatch(segment);
    }

    @Test
    public void autoTestCornersUseTheSameCurvatureDerivativeInPlannerAndRuntime() {
        GeometryFactory factory = new GeometryFactory();
        PathSegment segment = new HolonomicPathBuilder(Pose.zero(),
                factory.arcPose(30, 0, 7), factory.arcPose(30, -30, 7),
                factory.arcPose(-30, -30, 7), factory.arcPose(-30, 30, 7),
                factory.pose(30, 30, -90)).quickBuild().getParametricPath();
        assertCachedDerivativesMatch(segment);
    }

    private static void assertCachedDerivativesMatch(PathSegment segment) {
        for (PathPoint point : segment.getPointLUT()) {
            assertEquals("Curvature derivative at t=" + point.getT(),
                    segment.getCurvatureDerivative(point.getT()),
                    point.getCurvatureDerivative(), 1e-12);
        }
    }

    @Test
    public void closestPointProjectionFindsKnownNormalIntersection() {
        PathSegment segment = new PathSegment(new Parabola(1.0));
        double expectedT = 0.4;
        Vector point = segment.getPosition(expectedT);
        Vector normal = PathSegment.calculateArcNormal(
                segment.getFirstDerivative(expectedT),
                segment.getSecondDerivative(expectedT)
        );
        Vector query = point.plus(normal.times(3.0));

        assertEquals(expectedT, segment.getBestT(query), 1e-5);
    }

    @Test
    public void closestPointProjectionWorksForOppositeCurvature() {
        PathSegment segment = new PathSegment(new Parabola(-1.0));
        double expectedT = 0.65;
        Vector point = segment.getPosition(expectedT);
        Vector normal = PathSegment.calculateArcNormal(
                segment.getFirstDerivative(expectedT),
                segment.getSecondDerivative(expectedT)
        );
        Vector query = point.plus(normal.times(-2.0));

        assertEquals(expectedT, segment.getBestT(query), 1e-5);
    }

    private static final class Parabola implements ParametricSegment {
        private final double direction;

        private Parabola(double direction) { this.direction = direction; }

        @Override
        public Vector getPosition(double t) {
            return Vector.of(20.0 * t, direction * 10.0 * t * t, DistUnit.IN);
        }

        @Override
        public Vector getFirstDerivative(double t) {
            return Vector.of(20.0, direction * 20.0 * t, DistUnit.IN);
        }

        @Override
        public Vector getSecondDerivative(double t) {
            return Vector.of(0.0, direction * 20.0, DistUnit.IN);
        }
    }
}
