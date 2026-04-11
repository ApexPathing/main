package followers.quintic;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseBuilder;

/**
 * Fluent builder for {@link QuinticPath} objects.
 * Wraps {@link QuinticPath.Builder} with unit conversion and mirroring support via {@link PoseBuilder}.
 * Usage:
 *   QuinticPath path = new PathBuilder()
 *       .addPoint(0,  0,  0)
 *       .addPoint(24, 24, 90)
 *       .addPoint(48, 0,  0)
 *       .buildCommand(follower);
 * @author Sohum Arora 22985 Paraducks
 */
public class PathBuilder {

    private final QuinticPath.Builder builder = new QuinticPath.Builder();
    private final PoseBuilder poseBuilder;
    private double holdAtEndMs = 0;

    // region Constructors
    /**
     * Constructor for {@link PathBuilder} with specified units and mirroring
     * @param distanceUnit the {@link Distance.Units} to use for waypoint positions
     * @param angleUnit the {@link Angle.Units} to use for waypoint headings
     * @param mirror whether to mirror all poses across the y-axis
     */
    public PathBuilder(Distance.Units distanceUnit, Angle.Units angleUnit, boolean mirror) {
        this.poseBuilder = new PoseBuilder(distanceUnit, angleUnit, mirror);
    }

    /**
     * Constructor for {@link PathBuilder} with specified units and mirroring set to false
     * @param distanceUnit the {@link Distance.Units} to use for waypoint positions
     * @param angleUnit the {@link Angle.Units} to use for waypoint headings
     */
    public PathBuilder(Distance.Units distanceUnit, Angle.Units angleUnit) {
        this(distanceUnit, angleUnit, false);
    }

    /**
     * Constructor for {@link PathBuilder} with default units of inches and degrees,
     * and mirroring set to false
     */
    public PathBuilder() {
        this(Distance.Units.INCHES, Angle.Units.DEGREES, false);
    }
    // endregion

    // region Builder methods
    /**
     * Add a waypoint from a pre-built {@link Pose}
     * @param pose the {@link Pose} to add as a waypoint
     * @return this builder for chaining
     */
    public PathBuilder addPose(Pose pose) {
        builder.addPose(pose);
        return this;
    }

    /**
     * Add a waypoint from a pre-built {@link Pose} with a curvature hint
     * @param pose the {@link Pose} to add as a waypoint
     * @param curvature the curvature hint at this waypoint
     * @return this builder for chaining
     */
    public PathBuilder addPose(Pose pose, double curvature) {
        builder.addPose(pose, curvature);
        return this;
    }

    /**
     * Add a waypoint with x, y, and heading values in the builder's units
     * @param x the x component of the waypoint position
     * @param y the y component of the waypoint position
     * @param heading the heading at this waypoint in the builder's angle unit
     * @return this builder for chaining
     */
    public PathBuilder addPoint(double x, double y, double heading) {
        builder.addPose(poseBuilder.build(x, y, heading));
        return this;
    }

    /**
     * Add a waypoint with x, y, heading, and curvature in the builder's units
     * @param x the x component of the waypoint position
     * @param y the y component of the waypoint position
     * @param heading the heading at this waypoint in the builder's angle unit
     * @param curvature the curvature hint at this waypoint
     * @return this builder for chaining
     */
    public PathBuilder addPoint(double x, double y, double heading, double curvature) {
        builder.addPose(poseBuilder.build(x, y, heading), curvature);
        return this;
    }

    /**
     * Set how long the robot should hold at the end of the path before the command finishes
     * @param ms the time in milliseconds to hold at the end of the path
     * @return this builder for chaining
     */
    public PathBuilder holdAtPathEnd(double ms) {
        this.holdAtEndMs = ms;
        return this;
    }

    /**
     * Build the {@link QuinticPath} from the added waypoints
     * @return a new {@link QuinticPath} object
     */
    public QuinticPath build() {
        return builder.build();
    }

    /**
     * Build a {@link SequentialCommandGroup} that follows the path and optionally holds at the end
     * @param follower the {@link QuinticFollower} to use for path following
     * @return a {@link SequentialCommandGroup} containing the path command and optional wait
     */
    public SequentialCommandGroup buildCommand(QuinticFollower follower) {
        if (holdAtEndMs > 0) {
            return new SequentialCommandGroup(
                    new PathCommand(follower, build()),
                    new WaitCommand((long) holdAtEndMs)
            );
        }
        return new SequentialCommandGroup(new PathCommand(follower, build()));
    }
    // endregion

    // region Getters
    /** @return the hold time in milliseconds at the end of the path */
    public double getHoldAtEndMs() { return holdAtEndMs; }

    /** @return the {@link Distance.Units} used by the builder */
    public Distance.Units getDistanceUnit() { return poseBuilder.getDistanceUnit(); }

    /** @return the {@link Angle.Units} used by the builder */
    public Angle.Units getAngleUnit() { return poseBuilder.getAngleUnit(); }

    /** @return whether the builder is set to mirror poses across the y-axis */
    public boolean isMirror() { return poseBuilder.isMirror(); }
    // endregion
}