package paths.movements;

import core.FollowerConstants;
import feedforward.FFLut;
import feedforward.generators.TurnProfileGenerator;
import geometry.Angle;
import geometry.Pose;
import paths.Callback;

/**
 * Represents a stationary point-turn movement. The robot will remain at its starting (x, y)
 * coordinates and rotate to the specified target heading.
 *
 * @author DrPixelCat - 7842 alum
 * @author Sohum Arora - 22985 Paraducks
 */
public class Turn extends FollowerMovement {
    private FollowerConstants.Profile driveProfile;
    public FollowerConstants.Profile getDriveProfile() { return driveProfile; }
    public void setDriveProfile(FollowerConstants.Profile profile) { driveProfile = profile; }
    private final Pose startPose;
    private FFLut FFLut;
    private final double angularVelLimitRad;
    private final double angularAccelLimitRad;

    /**
     * Constructs a Turn movement.
     *
     * @param startPose The robot's state at the beginning of the turn.
     * @param targetHeading The final angle the robot should face.
     */
    public Turn(Pose startPose, Angle targetHeading) {
        this(startPose, targetHeading, Double.NaN, Double.NaN);
    }

    public Turn(Pose startPose, Angle targetHeading, double angularVelLimitRad,
                double angularAccelLimitRad) {
        this.startPose = startPose;
        this.endPose = new Pose(startPose.getVec(), targetHeading);
        this.angularVelLimitRad = angularVelLimitRad;
        this.angularAccelLimitRad = angularAccelLimitRad;
    }

    /** Attaches a heading callback to this traversal and returns the turn for chaining. */
    public Turn addAngularCallback(Angle heading, Runnable action) {
        addCallback(new Callback(heading, action));
        return this;
    }

    public Pose getStartPose() { return startPose; }

    public FFLut getFeedforwardLut() { return FFLut; }

    public void setFeedforwardLut(FFLut FFLut) { this.FFLut = FFLut; }

    @Override
    public Turn reversed() {
        Turn result = new Turn(endPose, startPose.getHeading(), angularVelLimitRad,
                angularAccelLimitRad);
        result.setDriveProfile(driveProfile);
        if (FFLut != null) {
            FollowerConstants constants = FollowerConstants.getInstance();
            if (driveProfile != null) { constants = constants.forProfile(driveProfile); }
            double velocityLimit = Double.isFinite(angularVelLimitRad)
                    ? angularVelLimitRad : constants.angularVelLimitRad;
            double accelerationLimit = Double.isFinite(angularAccelLimitRad)
                    ? angularAccelLimitRad : constants.angularAccelLimitRad;
            result.setFeedforwardLut(new TurnProfileGenerator(
                    velocityLimit, accelerationLimit, constants).generate(result));
        }
        return result;
    }
}
