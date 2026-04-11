package followers;

import drivetrains.Drivetrain;
import localizers.Localizer;
import followers.constants.P2PFollowerConstants;

import util.Angle;
import util.Pose;
import util.Vector;

/**
 * Simple point-to-point follower
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class P2PFollower extends Follower {
    private final P2PFollowerConstants constants;

    /**
     * Constructor for the P2PFollower
     * @param drivetrain the mecanum drivetrain class to control
     * @param localizer the Pinpoint localizer to get pose estimates from
     */
    public P2PFollower(P2PFollowerConstants constants, Drivetrain drivetrain, Localizer localizer) {
        this.constants = constants;
        this.drivetrain = drivetrain;
        this.localizer = localizer;
    }

    /**
     * Set the target pose for the robot to move to
     * @param targetPose the new target pose
     */
    public void setTargetPose(Pose targetPose) {
        super.setTargetPose(targetPose); // Use the unexposed method from the Follower class
    }

    @Override
    public void update() {
        localizer.update();
        Pose pose = localizer.getPose(); // Inches and radians

        if (pose == null || targetPose == null) {
            return;
        }

        Pose errorPose = targetPose.subtract(pose);
        double headingError = Math.abs(errorPose.getHeadingComponent().get(Angle.Units.RADIANS));
        double dist = targetPose.distanceTo(pose);

        if (dist < constants.translationalTolerance && headingError < constants.headingTolerance) {
            drivetrain.stop();
            isBusy = false;
            targetPose = null; // Clear target pose to prevent further movement until a new target is set
            return;
        }

        double dx = errorPose.getX();
        double dy = errorPose.getY();

        Vector error = new Vector(dx, dy);
        error.rotate(-pose.getHeading());

        double x = error.getX() * constants.translationalKp;
        double y = error.getY() * constants.translationalKp;
        double turn = headingError * constants.headingKp;

        double mag = Math.hypot(x, y);
        if (mag > constants.maxPower) {
            x /= mag;
            y /= mag;
        }

        if (mag > 0) {
            x = applyMinPower(x);
            y = applyMinPower(y);
        }

        turn = clip(turn, -constants.maxPower, constants.maxPower);
        drivetrain.drive(x, y, turn, 0); // Heading = 0 for robot centric drive
    }

    /**
     * Method to apply minimum power to a value to ensure the robot can overcome static friction and start moving
     * @param val the value to apply minimum power to
     * @return the value with minimum power applied
     */
    private double applyMinPower(double val) {
        if (Math.abs(val) < constants.minPower) {
            return Math.signum(val) * constants.minPower;
        }
        return val;
    }

    /**
     * Method to clip a value between a minimum and maximum value
     * @param val the value to clip
     * @param min the minimum value to clip val to
     * @param max the maximum value to clip val to
     * @return the clipped value
     */
    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
