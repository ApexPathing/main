package org.firstinspires.ftc.teamcode.tuning.manual;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import drivetrains.Drivetrain;
import followers.BSplineFollower;
import followers.constants.BSplineFollowerConstants;
import localizers.Localizer;
import paths.Path;
import paths.PathBuilder;
import util.Pose;

/**
 * OpMode for tuning the BSpline follower with Panels. Hold X to execute the test path forward,
 * and hold A to reset and drive back to the start position. Adjust the translation proportional
 * gain, heading proportional gain, and velocity feedforward in Panels.
 * @author Sohum Arora - 22985 Paraducks
 */
@Configurable
@TeleOp(name = "BSpline Tuner", group = "Apex Pathing Tuning")
public class BSplineTuner extends OpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;
    private BSplineFollower follower;
    private BSplineFollowerConstants followerConstants;
    private JoinedTelemetry fullTelem;

    private Path currentPath;
    private boolean pathActive = false;

    public static double translationP;
    public static double headingP;
    public static double velocityFF;

    public static double headingTolerance;
    public static double distanceTolerance;
    public static double tTolerance;

    @Override
    public void init() {
        Constants constants = new Constants();

        fullTelem = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);


        follower = (BSplineFollower) constants.build(hardwareMap, Pose.zero());


        followerConstants = new BSplineFollowerConstants();
        translationP = followerConstants.translationP;
        headingP = followerConstants.headingP;
        velocityFF = followerConstants.velocityFF;
        headingTolerance = followerConstants.headingTolerance;
        distanceTolerance = followerConstants.distanceTolerance;
        tTolerance = followerConstants.tTolerance;

        fullTelem.addLine(
                "Hold X to run the 48-inch multi-stage B-Spline test path, or hold A to force return home."
        );
        fullTelem.update();
    }

    @Override
    public void loop() {
        localizer.update();

        // Dynamically re-inject updated Panels data back into the operational constants object every loop
        followerConstants.translationP = translationP;
        followerConstants.headingP = headingP;
        followerConstants.velocityFF = velocityFF;
        followerConstants.headingTolerance = headingTolerance;
        followerConstants.distanceTolerance = distanceTolerance;
        followerConstants.tTolerance = tTolerance;

        if (gamepad1.x) {
            if (!pathActive) {
                // Construct a dynamic multi-stage curve utilizing line segments and hold targets
                currentPath = new PathBuilder(localizer.getPose())
                        .lineTo(new Pose(48, 0, 0))
                        .holdPose(1.5)
                        .build();
                follower.followPath(currentPath);
                pathActive = true;
            }
            follower.update();
        } else if (gamepad1.a) {
            if (!pathActive) {
                // Fallback safe path construction directing the tracking loop home
                currentPath = new PathBuilder(localizer.getPose())
                        .lineTo(new Pose(0, 0, 0))
                        .build();
                follower.followPath(currentPath);
                pathActive = true;
            }
            follower.update();
        } else {
            // Safe fallback sequence clearing operational active paths to protect drive system
            follower.stop();
            drivetrain.stop();
            pathActive = false;
        }

        // Handle path tracking termination notifications through standard hardware feedback loops
        if (pathActive && !follower.isBusy()) {
            gamepad1.rumble(0.5, 0.5, 100);
            gamepad1.setLedColor(0, 1, 0, 300);
            pathActive = false;
        } else if (pathActive) {
            gamepad1.setLedColor(1, 0, 0, 100);
        }

        fullTelem.addData("Robot X: ", localizer.getPose().getX());
        fullTelem.addData("Robot Y: ", localizer.getPose().getY());
        fullTelem.addData("Robot Heading (Deg): ", Math.toDegrees(localizer.getPose().getHeading()));
        fullTelem.addData("Follower Busy: ", follower.isBusy());
        fullTelem.addData("Drivetrain Output: ", drivetrain.toString());
        fullTelem.update();
    }
}
