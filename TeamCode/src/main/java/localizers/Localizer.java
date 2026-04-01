package localizers;

import util.Pose;

/**
 * Abstract Localizer class that all localizers should extend. This is used to define the basic structure of a localizer and to provide some common functionality.
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public abstract class Localizer {
    public Pose currentPosition = new Pose(0.0, 0.0, 0.0);
    public Pose currentVelocity = new Pose(0.0, 0.0, 0.0);

    /**
     * Gets the current robot pose (position and heading).
     * @return the current pose of the robot as a Pose object (x, y, heading)
     */
    public Pose getPose() { return currentPosition; }

    /**
     * Gets the current velocity vector. This is useful for path following and other advanced control algorithms.
     * @return the current velocity of the robot as a Vector (x, y)
     */
    public Pose getVelocity() { return currentVelocity; }

    /**
     * Sets the current pose of the robot.
     * @param pose the new pose of the robot
     */
    public abstract void setPose(Pose pose);

    /**
     * Updates the localizer's position, velocity, and acceleration estimates. This should be called in a loop to continuously update the robot's position.
     */
    public abstract void update();
}
