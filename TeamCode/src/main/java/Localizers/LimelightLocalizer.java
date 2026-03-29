package Localizers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import Util.Pose;
import Util.Vector;

// TODO: Add LimelightConstants class for pipeline numbers, camera offsets, etc.
/**
 * Limelight Class localizer for apriltags rewritten for safety and accuracy.
 * @author Krish Joshi - 26192 Heatwaves
 * @author Xander Haemel - 31616 404 not found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class LimelightLocalizer extends Localizer {
    private final Limelight3A limelight3A;
    private LLResult limelightResult;
    private Pose lastKnownPose = new Pose(0, 0, 0);

    private static final double METERS_TO_INCHES = 39.3701;

    public LimelightLocalizer(HardwareMap hardwareMap, String limelightName) {
        limelight3A = hardwareMap.get(Limelight3A.class, limelightName);
    }

    /**
     * Switch the limelight pipeline.
     * @param pipelineNumber The pipeline index to switch to.
     */
    public void switchPipeline(int pipelineNumber) {limelight3A.pipelineSwitch(pipelineNumber); }

    /**
     * Get fiducial results from the latest capture.
     * @return the List of AprilTags with their respective IDs.
     */
    public List<LLResultTypes.FiducialResult> getTagIDs() {
        if (limelightResult != null) {
            return limelightResult.getFiducialResults();
        }
        return null;
    }

    /**
     * Updates the robot orientation for Megatag2.
     * @param headingDegrees The current robot heading in degrees.
     */
    public void updateHeadingForMT2(double headingDegrees) {
        limelight3A.updateRobotOrientation(headingDegrees);
    }

    /**
     * Returns the total latency (capture + pipeline/targeting + staleness) in milliseconds.
     * @return Total vision latency in ms.
     */
    public double getLatencyMs() {
        if (limelightResult != null) {
            return limelightResult.getCaptureLatency() + limelightResult.getTargetingLatency() + limelightResult.getStaleness();
        }
        return 0;
    }

    @Override
    public void update() {
        limelightResult = limelight3A.getLatestResult();
        if (limelightResult != null && limelightResult.isValid()) {
            Pose3D botPose = limelightResult.getBotpose_MT2();
            if (botPose != null) {
                double x = botPose.getPosition().x * METERS_TO_INCHES;
                double y = botPose.getPosition().y * METERS_TO_INCHES;
                double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);
                lastKnownPose = new Pose(x, y, heading);
            }
        }
    }

    @Override
    public Pose getPose() { return lastKnownPose; }

    @Override
    public void setPose(Pose pose) { this.lastKnownPose = pose; }

    // TODO: Add velocity support to Limelight
    @Override
    public Pose getVelocity() { return new Pose(0, 0, 0); }
}
