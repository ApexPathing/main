package Followers;

import Util.Pose;

/**
 * Parent class for followers
 * @author Xander Haemel 31616 404 Not Found
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public abstract class Follower {
    protected boolean isBusy;
    protected Pose pose;

    /**
     * Update loop for the follower, should be called in a loop to update the follower's movement
     */
    public abstract void update();

    /**
     * Checks if the follower is still moving towards the target pose
     * @return true if the follower is still moving towards the target pose, false if it has reached the target pose
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * Get the robot's current pose estimate
     * @return the robot's current pose estimate
     */
    public Pose getPose() {
        return pose;
    }
}