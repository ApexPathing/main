package paths.movements;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import core.FollowerConstants;
import drivetrains.BaseDrivetrain;
import feedforward.FFLut;
import feedforward.MotionParameters;
import feedforward.generators.BaseProfileGenerator;
import feedforward.generators.MecanumProfileGenerator;
import feedforward.generators.SwerveProfileGenerator;
import feedforward.generators.TankProfileGenerator;
import geometry.Angle;
import geometry.AngleUnit;
import geometry.Dist;
import geometry.ParametricSegment;
import geometry.PathPoint;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.Callback;
import paths.constraint.AngularConstraint;
import paths.constraint.PathConstraint;
import paths.constraint.PathConstraint.Type;
import paths.constraint.TranslationalConstraint;
import paths.heading.HeadingInterpolator;
import paths.heading.ReversedHeadingInterpolator;

/**
 * Represents a complete, navigable geometric route for the robot to follow.
 *
 * <p>A {@code Path} encapsulates a continuous parametric curve (e.g., a B-Spline), its associated
 * heading interpolation strategy, and any scheduled mechanical callbacks triggered along the route.
 *
 * @author DrPixelCat - 7842 alum
 * @author Sohum Arora - 22985 Paraducks
 */
public class Path extends FollowerMovement {
    public enum PathType { HOLONOMIC, TANK }
    public enum BuildMode { QUICK, PROFILED }

    private final List<String> buildWarnings = new ArrayList<String>();
    private final ArrayList<PathConstraint> constraints = new ArrayList<PathConstraint>();
    private final PathType pathType;

    private PathSegment parametricPath;
    private HeadingInterpolator interpolator;
    private FFLut FFLut;
    private boolean isAccelBoosted = false;
    private BuildMode buildMode = BuildMode.QUICK;
    private boolean reverseConstraintProgress;

    /**
     * Creates a path object for the robot to follow
     *
     * @param pathType {@link PathType}: HOLONOMIC, or TANK
     */
    public Path(PathType pathType) { this.pathType = pathType; }

    /** Attaches a distance callback to this traversal and returns the path for chaining. */
    public Path addDistanceCallback(double progress, Runnable action) {
        addCallback(new Callback(progress, action));
        return this;
    }

    /** Attaches a heading callback to this traversal and returns the path for chaining. */
    public Path addAngularCallback(Angle heading, Runnable action) {
        addCallback(new Callback(heading, action));
        return this;
    }

    /** @param constraint The kinematic constraint to apply */
    public void addConstraint(PathConstraint constraint) { constraints.add(constraint); }

    /** @return the path's kinematic constraints */
    public PathConstraint[] getConstraints() { return constraints.toArray(new PathConstraint[0]); }

    /**
     * Evaluates the active velocity constraints for unprofiled quick builds. Acts as a step
     * function: the velocity limit changes when the robot crosses a constraint's 't' threshold.
     *
     * @param t The current path percentage [0.0, 1.0].
     * @param defaultLimit The hardware maximum velocity from FollowerConstants.
     * @return The active velocity limit in inches per second.
     */
    public double getQuickVelocityLimit(double t, double defaultLimit) {
        double constraintProgress = constraintProgress(t);
        double currentLimit = defaultLimit;
        double highestS = -1.0;

        for (PathConstraint baseConstraint : constraints) {
            if (baseConstraint instanceof TranslationalConstraint) {
                TranslationalConstraint constraint = (TranslationalConstraint) baseConstraint;
                if (constraint.getType() == Type.VELOCITY) {
                    if (constraintProgress >= constraint.getS() &&
                            constraint.getS() > highestS) {
                        currentLimit = constraint.getValueIn();
                        highestS = constraint.getS();
                    }
                }
            }
        }
        return currentLimit;
    }

    /** @param endPose The final target pose of this path. */
    public void setEndPose(Pose endPose) { this.endPose = endPose; }

    /** @return The generated LUT points from the ParametricPath */
    public PathPoint[] getGeneratedPoints() { return parametricPath.getPointLUT().clone(); }

    /**
     * Injects the calculated geometric curve (e.g., a B-Spline) that defines
     * the physical route the robot will drive.
     *
     * @param path The compiled path segment.
     */
    public void setParametricPath(PathSegment path) { this.parametricPath = path; }

    /**
     * Retrieves the geometric curve defining the physical route.
     *
     * @return The compiled path segment.
     */
    public PathSegment getParametricPath() { return parametricPath; }

    /** @param interpolator The heading interpolator to use */
    public void setInterpolator(HeadingInterpolator interpolator) {
        this.interpolator = interpolator;
    }

    /** @return The heading interpolator. */
    public HeadingInterpolator getInterpolator() { return interpolator; }

    /** @return The type of path: HOLONOMIC, or TANK */
    public PathType getPathType() { return pathType; }

    /** @return The feedforward look-up table */
    public FFLut getFeedforwardLut() { return FFLut; }

    /** @param FFLut The path's motion profile as a {@link FFLut} */
    public void setFeedforwardLut(FFLut FFLut) { this.FFLut = FFLut; }

    /** @return true if built with profiledBuild(), false if built with quickBuild() */
    public boolean isProfiled() { return FFLut != null; }

    public BuildMode getBuildMode() { return buildMode; }

    public void setBuildMode(BuildMode buildMode) { this.buildMode = buildMode; }

    /** Maps traversal progress into the coordinate system used by the constraint schedule. */
    public double constraintProgress(double traversalProgress) {
        return reverseConstraintProgress ? 1.0 - traversalProgress : traversalProgress;
    }

    /** Determines if this path should be followed with boosted acceleration. */
    public void useBoostedAccel() { isAccelBoosted = true; }

    /** @return Whether the path should be followed with boosted acceleration or not */
    public boolean isAccelBoosted() { return isAccelBoosted; }

    /**
     * Logs a non-fatal warning generated during the path building process (e.g., missing headings,
     * ignored waypoints. Duplicate warnings are ignored to prevent telemetry spam on the driver
     * station.
     *
     * @param warning The warning string.
     */
    public void addWarning(String warning) {
        if (!buildWarnings.contains(warning)) {
            buildWarnings.add(warning);
        }
    }

    /** @return A read-only list of warning strings. */
    public List<String> getWarnings() { return Collections.unmodifiableList(buildWarnings); }

    /**
     * Returns a fresh path over the same field geometry in the opposite direction. Callbacks are
     * deliberately not copied because they are side effects belonging to a specific traversal.
     */
    @Override
    public Path reversed() {
        final PathSegment sourceSegment = parametricPath;
        PathSegment reversedSegment = new PathSegment(new ParametricSegment() {
            @Override
            public Vector getPosition(double t) {
                return sourceSegment.getPosition(1.0 - t);
            }

            @Override
            public Vector getFirstDerivative(double t) {
                return sourceSegment.getFirstDerivative(1.0 - t).times(-1.0);
            }

            @Override
            public Vector getSecondDerivative(double t) {
                return sourceSegment.getSecondDerivative(1.0 - t);
            }
        });

        Path result = new Path(pathType);
        result.setParametricPath(reversedSegment);
        result.setInterpolator(new ReversedHeadingInterpolator(
                interpolator, sourceSegment.getLengthIn(),
                sourceSegment.getFirstDerivative(1.0)));

        Vector endTangent = reversedSegment.getFirstDerivative(1.0);
        Angle endHeading = result.getInterpolator().getHeadingTarg(
                0.0, endTangent, endTangent);
        result.setEndPose(new Pose(reversedSegment.getPosition(1.0), endHeading));

        for (PathConstraint constraint : constraints) {
            result.addConstraint(copyConstraint(constraint));
        }
        result.reverseConstraintProgress = !reverseConstraintProgress;
        result.isAccelBoosted = isAccelBoosted;
        result.buildMode = buildMode;
        for (String warning : buildWarnings) { result.addWarning(warning); }
        result.rebuildMotionProfile();
        return result;
    }

    /** Regenerates the profile appropriate to this path's drivetrain and build mode. */
    public void rebuildMotionProfile() {
        FollowerConstants constants = FollowerConstants.getInstance();
        if (pathType == PathType.TANK) {
            TankProfileGenerator generator = new TankProfileGenerator(constants, this);
            if (buildMode == BuildMode.QUICK) {
                setFeedforwardLut(generator.generateQuick(constants));
            } else {
                setFeedforwardLut(new FFLut(generator.generate()));
            }
            return;
        }

        if (buildMode == BuildMode.QUICK) {
            setFeedforwardLut(null);
            return;
        }

        BaseProfileGenerator generator = constants.drivetrainType ==
                BaseDrivetrain.DrivetrainType.COAXIAL_SWERVE
                ? new SwerveProfileGenerator(constants, this)
                : new MecanumProfileGenerator(constants, this);
        MotionParameters[] profile = generator.generate();
        if (hasUsableStartup(profile)) {
            setFeedforwardLut(new FFLut(profile));
        } else {
            setFeedforwardLut(null);
            addWarning("APEX WARNING: Motion profile contained no movement; falling back " +
                    "to closed-loop quick following.");
        }
    }

    private static boolean hasUsableStartup(MotionParameters[] profile) {
        if (profile == null || profile.length == 0) { return false; }
        MotionParameters start = profile[0];
        return start != null && (Math.abs(start.getTangentialVel()) > 1e-6 ||
                Math.abs(start.getTangentialAccel()) > 1e-6);
    }

    private static PathConstraint copyConstraint(PathConstraint constraint) {
        if (constraint instanceof TranslationalConstraint) {
            TranslationalConstraint translational = (TranslationalConstraint) constraint;
            return new TranslationalConstraint(translational.getS(), translational.getType(),
                    Dist.fromIn(translational.getValueIn()));
        }
        if (constraint instanceof AngularConstraint) {
            AngularConstraint angular = (AngularConstraint) constraint;
            return new AngularConstraint(angular.getS(), angular.getType(),
                    Angle.of(angular.getValueRad(), AngleUnit.RAD));
        }
        throw new IllegalArgumentException("Unsupported path constraint type: " +
                constraint.getClass().getName());
    }
}
