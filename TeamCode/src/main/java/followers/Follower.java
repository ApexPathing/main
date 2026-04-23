package followers;

import drivetrains.Drivetrain;
import followers.constants.FollowerConstants;
import followers.constants.P2PFollowerConstants;
import localizers.Localizer;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * Parent class for followers
 * @author Xander Haemel 31616 404 Not Found
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public abstract class Follower {
    protected Drivetrain drivetrain;
    protected Localizer localizer;

    protected boolean isBusy;

    protected Pose targetPose;

    /**
     * Constructor for the Follower class
     * Every Follower should take FollowerConstants, a Drivetrain, and a Localizer as parameters
     * Here is an example constructor for a P2PFollower, the same structure should be used for all.
     *
     * <pre>
     * {@code
     * public P2PFollower(P2PFollowerConstants constants, Drivetrain drivetrain, Localizer localizer) {
     *     super(drivetrain, localizer);
     *     this.constants = constants;
     * }
     * }
     * </pre>
     *
     * @param drivetrain the drivetrain to control
     * @param localizer the localizer to get pose estimates from
     */
    public Follower(Drivetrain drivetrain, Localizer localizer) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
    }

    /**
     * Update loop for the follower, should be called in a loop to update the follower's movement
     */
    public abstract void update();

    /**
     * Drives the robot using the provided joystick inputs and robot heading. The joystick inputs are adjusted
     * for field-centric or robot-centric control based on the constants, and a deadzone is applied to prevent drift.
     * @param x the left/right joystick input (positive for right, negative for left)
     * @param y the forward/backward joystick input (positive for forward, negative for backward)
     * @param turn the rotation joystick input (positive for clockwise, negative for counterclockwise)
     * @param robotHeading the current heading of the robot in radians, not used for robot centric control
     */
    public void drive(double x, double y, double turn, double robotHeading) {
        drivetrain.drive(x, y, turn, robotHeading);
    }

    /**
     * Drives the robot using the provided joystick inputs.
     * Constants are ignored and robot centric is used because no heading is passed. A deadzone is applied to prevent drift.
     * @param x the left/right joystick input (positive for right, negative for left)
     * @param y the forward/backward joystick input (positive for forward, negative for backward)
     * @param turn the rotation joystick input (positive for clockwise, negative for counterclockwise)
     */
    public void drive(double x, double y, double turn) {
        drivetrain.drive(x, y, turn, 0);
    }

    /**
     * Stops the robot and aborts any active path following
     */
    public void stop() {
        drivetrain.stop();
        isBusy = false;
        targetPose = null; // Clear target pose to prevent further movement until a new target is set
    }

    /**
     * Checks if the follower is still moving towards the target pose
     * @return true if the follower is still moving towards the target pose, false if it has reached the target pose
     */
    public boolean isBusy() { return isBusy; }

    /**
     * Set the current pose of the robot (for starting pose or relocalization)
     * @param pose the current pose of the robot
     */
    public void setPose(Pose pose) { localizer.setPose(pose); }

    /**
     * Get the robot's current pose estimate
     * @return the robot's current pose estimate
     */
    public Pose getPose() { return localizer.getPose(); }

    /**
     * Set the target pose for the robot to move to
     * (this isn't exposed to the user by default since some followers may use preplanned paths,
     * in which case the target pose would be set internally by the follower)
     * @param targetPose the new target pose
     */
    protected void setTargetPose(Pose targetPose) {
        isBusy = true;
        this.targetPose = targetPose.copy();
        this.targetPose.setDistanceUnit(Distance.Units.INCHES);
        this.targetPose.setAngleUnit(Angle.Units.RADIANS);
    }

    /**
     * Get the current target pose of the follower
     * @return the current target pose of the follower
     */
    public Pose getTargetPose() { return targetPose; }

    /**
     * Get the robot's current velocity estimate from the localizer
     * in a pose form (x and y components in the local robot frame, rotational component in radians per second)
     * @return the robot's current velocity estimate from the localizer
     */
    public Pose getVelocity() { return localizer.getVelocity(); }
}