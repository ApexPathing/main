package paths;

import java.util.ArrayList;
import java.util.List;

import paths.heading.HeadingInterpolator;
import util.Angle;
import util.Pose;

/**
 * Represents a complete, navigable route for the robot to follow.
 * <p>
 * A Path is composed of a sequential series of segments and their associated
 * heading strategies, wrapped together in {@link PathNode}s to guarantee they
 * remain synchronized during execution._
 * <p>
 * Author: DrPixelCat
 * @author Sohum Arora 22985 Paraducks
 */
public class Path {

    /**
     * A composite wrapper that securely binds a geometric path segment to its
     * corresponding heading interpolation strategy.
     */
    public enum NodeType {
        DRIVE,
        TURN,
        HOLD
    }

    public static class PathNode {
        public final NodeType type;

        // Populated if type == DRIVE
        public final PathSegment segment;
        public final HeadingInterpolator interpolator;

        // Populated if type == TURN
        public final Angle targetHeading;

        // Populated if type == HOLD
        public final Pose holdPose;
        public final double holdDurationSeconds;

        public PathNode(PathSegment segment, HeadingInterpolator interpolator) {
            this.type = NodeType.DRIVE;
            this.segment = segment;
            this.interpolator = interpolator;
            this.targetHeading = null;
            this.holdPose = null;
            this.holdDurationSeconds = 0.0;
        }

        public PathNode(Angle targetHeading) {
            this.type = NodeType.TURN;
            this.segment = null;
            this.interpolator = null;
            this.targetHeading = targetHeading;
            this.holdPose = null;
            this.holdDurationSeconds = 0.0;
        }

        public PathNode(Pose holdPose, double durationSeconds) {
            this.type = NodeType.HOLD;
            this.segment = null;
            this.interpolator = null;
            this.targetHeading = null;
            this.holdPose = holdPose;
            this.holdDurationSeconds = durationSeconds;
        }
    }

    private final List<PathNode> nodes = new ArrayList<>();
    private int currentIndex = 0;
    private final List<String> buildWarnings = new ArrayList<>();

    public void addSegment(PathSegment segment, HeadingInterpolator interpolator) {
        nodes.add(new PathNode(segment, interpolator));
    }

    public void overrideLastInterpolator(HeadingInterpolator interpolator) {
        if (nodes.isEmpty()) return;
        PathNode last = nodes.get(nodes.size() - 1);
        if (last.type == NodeType.DRIVE) {
            nodes.set(nodes.size() - 1, new PathNode(last.segment, interpolator));
        }
    }

    public void addTurn(Angle targetHeading) {
        nodes.add(new PathNode(targetHeading));
    }

    public void addHold(Pose holdPose, double durationSeconds) {
        nodes.add(new PathNode(holdPose, durationSeconds));
    }

    public PathNode getCurrentNode() {
        if (nodes.isEmpty()) throw new IllegalStateException("Path is empty!");
        return nodes.get(currentIndex);
    }

    /**
     * Advances the path's internal state to the next segment.
     * If the path is already on the last segment, this method does nothing._
     */
    public void advance() {
        if (!isLastSegment()) {
            currentIndex++;
        }
    }

    /**
     * Adds a per-path warning based on feedback from PathBuilder_
     *
     * @param warning The warning string to be displayed on the driver hub_
     */
    public void addWarning(String warning) {
        if (!buildWarnings.contains(warning)) { // Prevent spamming the exact same warning twice
            buildWarnings.add(warning);
        }
    }

    /**
     * Gets Path warnings to be displayed to driver_
     *
     * @return The list of warnings corresponding to each path segment._
     */
    public List<String> getWarnings() {
        return buildWarnings;
    }

    /**
     * Checks if the robot has reached the final segment of the path._
     *
     * @return True if the current segment is the last one in the list, false otherwise._
     */
    public boolean isLastSegment() {
        return currentIndex >= nodes.size() - 1;
    }

    /**
     * Resets the internal index back to zero so the path can be run again from the beginning._
     * This should be called immediately before...
     */
    public void reset() {
        currentIndex = 0;
    }
}
