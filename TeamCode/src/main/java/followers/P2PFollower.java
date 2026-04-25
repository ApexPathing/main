package followers;

import controllers.PDFLController;
import drivetrains.Drivetrain;
import localizers.Localizer;
import followers.constants.P2PFollowerConstants;

import util.Pose;
import util.Vector;

/**
 * Simple point-to-point follower
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class P2PFollower extends Follower {
    private final P2PFollowerConstants constants;

    private final PDFLController axialController;
    private final PDFLController strafeController;
    private final PDFLController headingController;

    /**
     * Constructor for the P2PFollower
     * @param drivetrain the mecanum drivetrain class to control
     * @param localizer the Pinpoint localizer to get pose estimates from
     */
    public P2PFollower(P2PFollowerConstants constants, Drivetrain drivetrain, Localizer localizer) {
        super(drivetrain, localizer);
        this.constants = constants;
        this.axialController = constants.axialController;
        this.strafeController = constants.strafeController;
        this.headingController = constants.headingController;
    }

    /**
     * Set the target pose for the robot to move to
     * @param targetPose the new target pose
     */
    public void setTargetPose(Pose targetPose) {
        this.axialController.reset();
        this.strafeController.reset();
        this.headingController.reset();
        super.setTargetPose(targetPose); // Use the unexposed method from the Follower class
    }

    public boolean axialAtTarget() { return constants.axialController.isAtTarget(); }

    public boolean strafeAtTarget() { return constants.strafeController.isAtTarget(); }

    public boolean headingAtTarget() { return constants.headingController.isAtTarget(); }

    @Override
    public void update() {
        localizer.update();

        if (!isBusy) {
            return; // No need to calculate anything if we're not busy
        }

        Pose pose = localizer.getPose();
        Vector translationError = targetPose.toVec().subtract(pose.toVec());
        double headingError = targetPose.getHeading() - pose.getHeading(); // Controller handles wrapping

        if (axialController.isAtTarget() && strafeController.isAtTarget() && headingController.isAtTarget()) {
            isBusy = false;
            drivetrain.stop();
            return;
        }

        // Note: powers are clipped to max powers defined in constants
        Vector translational = new Vector(
                axialController.calculate(translationError.getX()),
                strafeController.calculate(translationError.getY())
        ).rotated(-pose.getHeading()); // Rotate to the robot's frame of reference
        double turn = -headingController.calculate(headingError);

        drivetrain.drive(translational.getX(), translational.getY(), turn);
    }
}
