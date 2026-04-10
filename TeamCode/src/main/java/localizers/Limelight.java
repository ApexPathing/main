package localizers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import localizers.constants.LimelightConstants;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * Localizer for the Limelight Vision 3A Smart Camera using the Limelight3A class.
 * Supports AprilTag detection with MetaTag2.
 * @author Krish Joshi - 26192 Heatwaves
 * @author Xander Haemel - 31616 404 not found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Limelight extends Localizer {
    private final LimelightConstants constants;
    private final Limelight3A cam;
    private Pose lastPose = null;

    private ElapsedTime timer;
    private long lastTime;

    public Limelight(HardwareMap hardwareMap, LimelightConstants constants) {
        this.constants = constants;
        this.cam = hardwareMap.get(Limelight3A.class, constants.name);
        this.cam.pipelineSwitch(constants.pipeline);
        this.timer = new ElapsedTime();
    }

    @Override
    public void update() {
        LLResult result = cam.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose;
            if (constants.useMetaTag2) {
                botPose = result.getBotpose_MT2();
            } else {
                botPose = result.getBotpose(); // Standard ATag detection
            }

            if (botPose != null) {
                Pose newPose = new Pose(
                        botPose.getPosition().x, botPose.getPosition().y, // Meters
                        botPose.getOrientation().getYaw(AngleUnit.RADIANS),
                        Distance.Units.METERS, Angle.Units.RADIANS, false
                );

                // Get current time in nanoseconds
                long currentTime = timer.nanoseconds();

                // Velocity calculation
                if (lastPose != null) {
                    double dt = (currentTime - lastTime) / 1e9; // Convert nanoseconds to seconds
                    if (dt > 0) {
                        Pose deltaPose = newPose.subtract(lastPose);

                        this.currentVelocity = new Pose(
                                deltaPose.getX() / dt, deltaPose.getY() / dt, // Meters per second
                                deltaPose.getHeading() / dt, // Radians per second
                                Distance.Units.METERS, Angle.Units.RADIANS, false
                        );
                    }
                }

                this.currentPose = newPose;
                this.lastPose = newPose;
                this.lastTime = currentTime;
            }
        }
    }


    @Override
    public void setPose(Pose pose) {
        // The Limelight doesn't support setting position, but we can set the orientation
        cam.updateRobotOrientation(pose.getHeadingComponent().get(Angle.Units.DEGREES));
    }
}
