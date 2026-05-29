package paths.builders;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import paths.movements.Path;
import paths.callbacks.Callback;
import geometry.BSpline;
import geometry.PathSegment;
import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import geometry.Angle;
import geometry.Vector;
import geometry.ArcPose;
import geometry.Pose;

/**
 * A builder class designed to construct a {@link Path} fluently.
 * <p>
 * This class captures path configurations (waypoints, interpolators, callbacks)
 * in any order and defers geometric compilation until {@link #build()} is called.
 * C2 (tangent and acceleration) continuity is guaranteed in this builder.
 * <p>
 * @author DrPixelCat
 * @author Sohum Arora 22985 Paraducks
 */
public class PathBuilder {
    public Path path;

    // State Tracking
    private final Pose segmentStartPose;
    private Pose expectedEndPose;
    private Pose[] rawPoses = null;

    private InterpolationStyle currentStyle = InterpolationStyle.SMOOTH_START_TO_END;
    private HeadingInterpolator customInterpolator = null;

    // Stores callbacks to be validated and attached during the build process
    private final List<Runnable> buildTasks = new ArrayList<>();

    /**
     * Initializes the PathBuilder with the starting location and heading of the robot.
     *
     * @param startPose The initial Pose of the robot at the beginning of the path.
     */
    public PathBuilder(Pose startPose) {
        this.path = new Path();
        this.segmentStartPose = startPose;
    }

    /**
     * Stores a sequence of control points to define a continuous Uniform Cubic B-Spline.
     * Any {@link ArcPose} provided is dynamically split into two adjacent control points to round sharp corners.
     * <p>
     * Note: Geometric processing is deferred until {@link #build()} is called.
     *
     * @param poses A variable number of waypoints/control points.
     * @return The current PathBuilder instance for method chaining.
     * @throws IllegalArgumentException If endpoints are arc poses or insufficient points are provided.
     * @throws IllegalStateException If control points have already been added to this builder.
     */
    public PathBuilder addControlPoints(Pose... poses) {
        if (this.rawPoses != null) {
            throw new IllegalStateException("Control points have already been added to this builder!");
        }
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }

        this.rawPoses = poses;
        this.expectedEndPose = poses[poses.length - 1];

        return this;
    }

    /**
     * Overrides the default (SMOOTH_START_TO_END) interpolation with a different {@link InterpolationStyle}
     *
     * @param style The interpolation style to apply.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolateWith(InterpolationStyle style) {
        switch (style) {
            case TANGENT_OPTIMAL:
            case TANGENT_FORWARD:
            case SMOOTH_START_TO_END:
                path.addWarning("APEX WARNING: SMOOTH_START_TO_END is the default interpolator, there's no need to change it!");
                this.currentStyle = style;
                break;
            default:
                throw new IllegalArgumentException(
                        "You need more parameters for: " + style.name() + "!");
        }
        return this;
    }

    /**
     * Overrides the default (SMOOTH_START_TO_END) interpolation for one defined with a custom function
     * of distance percentage (s)
     *<p>
     * Example usage that spins the robot as a function of s^2:
     * </p>
     * .interpolateWith(s -> Angle.fromDeg(180 * (s * s))
     *
     * @param function The Angle function
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolateWith(Function<Double, Angle> function) {
        return interpolateWith(new HeadingInterpolator(function));
    }

    private PathBuilder interpolateWith(HeadingInterpolator interpolator) {
        this.customInterpolator = interpolator;
        return this;
    }

    /**
     * Attaches an executable callback based on the physical distance percentage.
     *
     * @param s The physical distance percentage [0.0, 1.0].
     * @param action The code to execute.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder addDistanceCallback(double s, Runnable action) {
        buildTasks.add(() -> path.addCallback(new Callback(s, action)));
        return this;
    }

    /**
     * Attaches an executable callback based on the robot reaching a target heading.
     * Safety checks ensure the angle is mathematically reachable.
     *
     * @param angle The Angle at which the callback should trigger.
     * @param action The code to execute.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder addAngularCallback(Angle angle, Runnable action) {
        // Defer validation to build() so expectedEndPose is guaranteed to be set regardless of call order
        //TODO: Fine for now, but add in safety checking for lambda overrides
        buildTasks.add(() -> {
            double startRad = segmentStartPose.getHeading().getRad();
            double endRad = expectedEndPose.getHeading().getRad();

            if (Double.isFinite(startRad) && Double.isFinite(endRad)) {
                double targetRad = angle.getRad();

                double totalDiff = getShortestAngularDifference(startRad, endRad);
                double targetDiff = getShortestAngularDifference(startRad, targetRad);

                if (Math.abs(totalDiff) < 1e-6) {
                    if (Math.abs(targetDiff) > 1e-6) {
                        throw new IllegalArgumentException("Angular callback out of bounds: The path's heading is constant.");
                    }
                } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                    throw new IllegalArgumentException("Angular callback is outside the range of the path's start and end headings.");
                }
            }
            path.addCallback(new Callback(angle, action));
        });

        return this;
    }

    /**
     * Compiles all configuration data, calculates new ctrl points from {@link ArcPose}, generates the curve,
     * verifies callback safety, and returns the completed executable Path.
     *
     * @return The fully constructed {@link Path} object ready for execution.
     */
    public Path build() {
        if (rawPoses == null) {
            throw new IllegalStateException("Cannot build path: No control points were added!");
        }

        // 1. Pre-process the points (Expand ArcPoses)
        ArrayList<Pose> processedPoses = new ArrayList<>(rawPoses.length * 2);
        processedPoses.add(rawPoses[0]);

        boolean intermediateWarningSent = false;

        for (int i = 1; i < rawPoses.length - 1; i++) {
            Pose currentPose = rawPoses[i];

            if (!intermediateWarningSent && Double.isFinite(currentPose.getHeading().getRad())) {
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are ignored! Only the " +
                        "final pose heading controls the end heading.");
                intermediateWarningSent = true;
            }

            if (currentPose instanceof ArcPose) {
                ArcPose arcPose = (ArcPose) currentPose;
                double radius = arcPose.getRadius().getIn();

                if (radius < 2.0) {
                    throw new IllegalArgumentException("ArcPose radius must be at least 2.0 inches.");
                }

                Pose prevPose = rawPoses[i - 1];
                Pose nextPose = rawPoses[i + 1];

                Vector vecToLast = prevPose.getPos().minus(arcPose.getPos());
                Vector vecToNext = nextPose.getPos().minus(arcPose.getPos());

                double distToLast = vecToLast.getMag().getIn();
                double distToNext = vecToNext.getMag().getIn();

                if (radius > distToLast) {
                    throw new IllegalArgumentException("ArcPose radius (" + radius + ") exceeds distance to the last control point.");
                } else if (radius > distToNext) {
                    throw new IllegalArgumentException("ArcPose radius (" + radius + ") exceeds distance to the next control point.");
                }

                Vector p1Vec = arcPose.getPos().plus(vecToLast.times(radius / distToLast));
                Vector p2Vec = arcPose.getPos().plus(vecToNext.times(radius / distToNext));

                processedPoses.add(new Pose(p1Vec, arcPose.getHeading()));
                processedPoses.add(currentPose);
                processedPoses.add(new Pose(p2Vec, arcPose.getHeading()));

            } else {
                processedPoses.add(currentPose);
            }
        }

        processedPoses.add(rawPoses[rawPoses.length - 1]);

        // 2. Build the curve using the fully processed points
        Vector[] vectors = new Vector[processedPoses.size() + 1];
        vectors[0] = segmentStartPose.getPos(); // Inherit end of previous segment

        for (int i = 0; i < processedPoses.size(); i++) {
            vectors[i + 1] = processedPoses.get(i).getPos();
        }

        PathSegment curve = new PathSegment(new BSpline(vectors));
        path.setParametricPath(curve);

        // 3. Inject interpolator state
        if (customInterpolator != null) {
            path.setInterpolator(customInterpolator);
        } else {
            path.setInterpolator(buildSafeInterpolator(segmentStartPose, expectedEndPose));
        }

        // 4. Run deferred tasks (validating boundaries and attaching callbacks)
        for (Runnable task : buildTasks) {
            task.run();
        }

        return path;
    }

    // region Helpers

    /**
     * Safely constructs a HeadingInterpolator, automatically falling back to TANGENT_FORWARD
     * and generating a warning if a user forgot to supply valid headings in their Poses.
     */
    private HeadingInterpolator buildSafeInterpolator(Pose start, Pose end) {
        if (currentStyle == InterpolationStyle.TANGENT_FORWARD) {
            return new HeadingInterpolator(InterpolationStyle.TANGENT_FORWARD);
        }

        boolean missingHeading = !Double.isFinite(start.getHeading().getRad()) || !Double.isFinite(end.getHeading().getRad());

        if (missingHeading) {
            path.addWarning("APEX WARNING: Segment missing start/end heading! Falling back to TANGENT_FORWARD. Use Pose(x, y, heading) to fix this.");
            return new HeadingInterpolator(InterpolationStyle.TANGENT_FORWARD);
        }

        return new HeadingInterpolator(currentStyle, start.getHeading(), end.getHeading());
    }

    private double getShortestAngularDifference(double from, double to) {
        double diff = (to - from) % (2 * Math.PI);
        if (diff > Math.PI) diff -= 2 * Math.PI;
        else if (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}