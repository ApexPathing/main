package localizers;

import util.Pose;

/**
 * Abstract Localizer class that all localizers should extend. This is used to define the basic structure of a localizer and to provide some common functionality.
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public abstract class Localizer {
    public Pose currentPose;
    public Pose currentVelocity;

    /**
     * Gets the current robot pose in inches and radians.
     * @return the current pose of the robot as a {@link Pose} object (x, y, heading)
     */
    public Pose getPose() { return currentPose; }

    /**
     * Gets the current vector velocities in inches and radians per second.
     * @return the current velocity as a {@link Pose} object (x vel, y vel, heading vel)
     */
    public Pose getVelocity() { return currentVelocity; }

    /**
     * Sets the current pose of the robot.
     * @param pose the new {@link Pose} of the robot
     */
    public abstract void setPose(Pose pose);

    /**
     * Updates the localizer's position and velocity estimates.
     * This should be called in a loop to continuously update the robot's position.
     */
    public abstract void update();
}
