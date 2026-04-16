package followers;

import com.qualcomm.robotcore.util.Range;

import controllers.PDFLController;
import controllers.VectorControllers.PDLVectorController;
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
    private boolean disable = true;

    private final PDLVectorController translationalController;
    private final PDFLController headingController;


    /**
     * Constructor for the P2PFollower
     * @param drivetrain the mecanum drivetrain class to control
     * @param localizer the Pinpoint localizer to get pose estimates from
     */
    public P2PFollower(P2PFollowerConstants constants, Drivetrain drivetrain, Localizer localizer) {
        this.constants = constants;
        this.drivetrain = drivetrain;
        this.localizer = localizer;

        this.translationalController = constants.translationalController;
        this.headingController = constants.headingController;
    }

    /**
     * Set the target pose for the robot to move to
     * @param targetPose the new target pose
     */
    public void setTargetPose(Pose targetPose) {
        disable = false;
        constants.headingController.reset();
        constants.translationalController.reset();
        super.setTargetPose(targetPose); // Use the unexposed method from the Follower class
    }

    @Override
    public void update() {
        localizer.update();
        Pose location = localizer.getPose();
        Vector translationError = targetPose.toVec().subtract(location.toVec());
        /* NOTE: Controller handles angleWrapping via headingController.useAsAngularController() in base */
        double headingError = targetPose.getHeading() - location.getHeading();

        if (disable) {
            drivetrain.stop();
            return;
        }

        // Replaced comparisons with controller methods
        if (constants.translationalController.isAtTarget() && constants.headingController.isAtTarget()) {
            disable = true;
            isBusy = false; // Reset busy state so OpModes can progress
            drivetrain.stop();
            return;
        }

        Vector drive = constants.translationalController.calculate(translationError).rotated(-location.getHeading());
        double turn = constants.headingController.calculate(headingError);

        if (drive.getMagnitudeSquared() > constants.maxPower * constants.maxPower) {
            drive = drive.normalize().multiply(constants.maxPower);
        }

        // Note: minimum power provided by controllers

        turn = Range.clip(turn, -constants.maxPower, constants.maxPower);
        // Map Vector X to Drive (Y) and Vector Y to Strafe (X) for Mecanum drive
        drivetrain.drive(drive.getY(), drive.getX(), turn, 0);
    }
}
