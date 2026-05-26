package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;

import followers.BSplineFollower;
import followers.constants.BSplineFollowerConstants;
import paths.ExamplePathAPIV3;
import paths.Path;
import util.Pose;

/**
 * Test Autonomous opMode utilizing {@link paths.ExamplePathAPIV3}
 * IMPORTANT: Make sure that you have your {@link BSplineFollowerConstants} set up
 */
@Autonomous(name = "Apex BSpline Auto Test", group = "Apex Pathing Tests")
public class BSplineAutoTest extends LinearOpMode {
    private boolean mirror = false; //Pass in the constructor for ExamplePathAPIV3, change as required
    @Override
    public void runOpMode() throws InterruptedException {
        BSplineFollower follower = (BSplineFollower) new Constants().build(hardwareMap, Pose.zero());
        Path autoPath = new ExamplePathAPIV3(mirror).testPath();

        while (opModeInInit()){
            telemetry.addLine("Robot initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        follower.followPath(autoPath);

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            Pose currentPose = follower.getPose();
            Pose targetPose = follower.getTargetPose();

            telemetry.addLine(follower.isBusy() ? "Follower IS busy" : "Follower is NOT busy");
            if (targetPose != null) {
                telemetry.addData("Target X", targetPose.getX());
                telemetry.addData("Target Y", targetPose.getY());
            }
            telemetry.addData("Current X", currentPose.getX());
            telemetry.addData("Current Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }

        follower.stop();
    }
}