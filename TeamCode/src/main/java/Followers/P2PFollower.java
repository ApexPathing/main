package Followers;

import Drivetrains.MecanumDrive;
import Localizers.PinpointLocalizer;
import Util.Pose;
import Util.Vector;

/**
 * Simple point-to-point follower for a mecanum drivetrain using a goBILDA Pinpoint Odometry Computer.
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class P2PFollower extends Follower {
    private final MecanumDrive drive;
    private final PinpointLocalizer localizer;

    private Pose targetPose;

    private Vector currentVector;

    private double translationalKp = 0.03;
    private double headingKp = 0.5;

    private double translationalTolerance = 1.0; // Inches
    private double headingTolerance = 0.05; // Radians

    private double maxPower = 1.0;
    private double minPower = 0.05;

    /**
     * Constructor for the P2PFollower
     * @param drive the mecanum drivetrain class to control
     * @param localizer the Pinpoint localizer to get pose estimates from
     */
    public P2PFollower(MecanumDrive drive, PinpointLocalizer localizer) {
        this.drive = drive;
        this.localizer = localizer;
    }

    /**
     * Constructor for the P2PFollower with custom coefficients
     * @param drive the mecanum drivetrain class to control
     * @param localizer the Pinpoint localizer to get pose estimates from
     * @param translationalKp the proportional coefficient for translational control
     * @param headingKp the proportional coefficient for heading control
     */
    public P2PFollower(MecanumDrive drive, PinpointLocalizer localizer, double translationalKp, double headingKp) {
        this.drive = drive;
        this.localizer = localizer;
        this.translationalKp = translationalKp;
        this.headingKp = headingKp;
    }

    /**
    * Method to set the coefficients for the follower
    * @param translationalKp the proportional coefficient for translational control
    * @param headingKp the proportional coefficient for heading control
    */
    public void setCoefficients(double translationalKp, double headingKp) {
        this.translationalKp = translationalKp;
        this.headingKp = headingKp;
    }

    /**
     * Set tolerances for the follower to consider itself at the target pose
     * @param translationalTolerance the distance tolerance for the follower to consider itself at the target pose
     * @param headingTolerance the heading tolerance for the follower to consider itself at the target pose
     */
    public void setTolerances(double translationalTolerance, double headingTolerance) {
        this.translationalTolerance = translationalTolerance;
        this.headingTolerance = headingTolerance;
    }

    /**
     * Set the maximum and minimum motor power
     * @param maxPower the maximum motor power to apply
     */
    public void setPowerLimits(double maxPower, double minPower) {
        this.maxPower = maxPower;
        this.minPower = minPower;
    }

    /**
     * Set the target pose for the robot to move to
     * @param newTargetPose the new target pose
     */
    public void setTargetPose(Pose newTargetPose) {
        isBusy = true;
        targetPose = newTargetPose;
    }

    /**
     * Get the current target pose of the follower
     * @return the current target pose of the follower
     */
    public Pose getTargetPose() {
        return targetPose;
    }

    @Override
    public void update() {
        localizer.update();
        pose = localizer.getPose();

        if (pose == null || targetPose == null) {
            return;
        }

        Pose errorPose = targetPose.subtract(pose);
        double dist = targetPose.distanceFrom(pose);
        double headingError = normalizeAngle(errorPose.getHeading());

        if (dist < translationalTolerance && Math.abs(headingError) < headingTolerance) {
            drive.botCentricDrive(0, 0, 0);
            isBusy = false;
            return;
        }

        double dx = errorPose.getX();
        double dy = errorPose.getY();

        Vector error = new Vector(dx, dy);
        error.rotateVec(-pose.getHeading());

        double x = error.getX() * translationalKp;
        double y = error.getY() * translationalKp;
        double turn = headingError * headingKp;
        currentVector = new Vector(x, y);

        double mag = Math.hypot(x, y);
        if (mag > maxPower) {
            x /= mag;
            y /= mag;
        }

        if (mag > 0) {
            x = applyMinPower(x);
            y = applyMinPower(y);
        }

        turn = clip(turn, -maxPower, maxPower);
        drive.botCentricDrive(x, y, turn);
    }

    /**
     * Method to apply minimum power to a value to ensure the robot can overcome static friction and start moving
     * @param val the value to apply minimum power to
     * @return the value with minimum power applied
     */
    private double applyMinPower(double val) {
        if (Math.abs(val) < minPower) {
            return Math.signum(val) * minPower;
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

    /**
     * Method to normalize an angle to the range [-pi, pi]
     * @param angle the angle to normalize
     * @return the normalized angle
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Get the current movement vector of the robot, where the x component is the forward/backward power and the y component is the strafe power
     * @return the current movement vector of the robot
     */
    public Vector getVector() {
        return currentVector;
    }
}
