package localizers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import localizers.constants.PinpointConstants;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * Localizer for the goBILDA Pinpoint Odometry Computer using the GoBildaPinpointDriver class.
 * @author Sohum Arora 22985
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class Pinpoint extends Localizer {
    private final GoBildaPinpointDriver pinpoint;

    public Pinpoint(HardwareMap hardwareMap, PinpointConstants constants, Pose startPose) {
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
        this.setPose(startPose); // Set starting position
    }

    @Override
    public void update() {
        pinpoint.update();
        currentPose = Pose.fromPose2D(
                pinpoint.getPosition(), Distance.Units.INCHES, Angle.Units.RADIANS, false
        );
        currentVelocity = new Pose(
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                pinpoint.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized()),
                Distance.Units.INCHES, Angle.Units.RADIANS, false
        );
    }

    @Override
    public void setPose(Pose pose) {
        pinpoint.setPosition(pose.toPose2D());
    }
}