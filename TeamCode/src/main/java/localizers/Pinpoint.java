package localizers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import localizers.constants.PinpointConstants;
import util.Pose;

/**
 * Localizer for the goBILDA Pinpoint Odometry Computer using the GoBildaPinpointDriver class.
 * @author Sohum Arora 22985
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class Pinpoint extends Localizer {
    private final PinpointConstants constants;

    private final GoBildaPinpointDriver pinpoint;
    private Pose2D currentPose;

    public Pinpoint(HardwareMap hardwareMap, PinpointConstants constants, Pose startPose) {
        this.constants = constants;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, constants.name);
        pinpoint.setOffsets(constants.xOffset, constants.yOffset, constants.distanceUnit); // Offsets
        if (constants.customEncoderResolution != 0) { // Encoder resolution
            pinpoint.setEncoderResolution(constants.customEncoderResolution, constants.distanceUnit);
        } else {
            pinpoint.setEncoderResolution(constants.encoderResolution);
        }
        pinpoint.setEncoderDirections(constants.xPodDirection, constants.yPodDirection); // Pod directions
        if (constants.yawScalar != 0) { // Yaw scalar
            pinpoint.setYawScalar(constants.yawScalar);
        }
        pinpoint.resetPosAndIMU(); // Reset IMU
        pinpoint.setPosition(Pose.toPose2D(startPose)); // Set starting position
    }

    @Override
    public void update() {
        pinpoint.update();
        currentPose = pinpoint.getPosition(); // Keep as Pose2D for now
    }

    @Override
    public Pose getPose() {
        return Pose.fromPose2D(currentPose); // Convert to Pose for consistency with the Localizer interface
    }

    @Override
    public Pose getVelocity() {
        return new Pose(
                pinpoint.getVelX(constants.distanceUnit),
                pinpoint.getVelY(constants.distanceUnit),
                pinpoint.getHeadingVelocity(constants.angleUnit.getUnnormalized())
        );
    }

    @Override
    public void setPose(Pose pose) {
        pinpoint.setPosition(Pose.toPose2D(pose));
    }
}