package followers.constants;

import drivetrains.Drivetrain;
import followers.Follower;
import localizers.Localizer;

/**
 * Abstract base class for follower constants.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class FollowerConstants {
    /**
    * Constructor for the FollowerConstants class
    */
    public FollowerConstants() { }

    /**
     * Builds a Follower instance based on the constants defined in this class.
     * This method must be implemented by subclasses to return the appropriate type of Follower.
     * @param drivetrain the Drivetrain to use for controlling the robot's movement
     * @param localizer the Localizer to use for tracking the robot's position and heading
     * @return a Follower instance configured with the constants defined in this class
     */
    public abstract Follower build(Drivetrain drivetrain, Localizer localizer);
}