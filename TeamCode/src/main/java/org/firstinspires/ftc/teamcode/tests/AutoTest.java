package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;

import followers.P2PFollower;
import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseBuilder;

/**
 * Test OpMode for using Apex Pathing in Autonomous mode.
 * Edit the poses array to test different types of movement individually or together.
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Autonomous(name = "Apex Autonomous Test", group = "Apex Pathing Tests")
public class AutoTest extends LinearOpMode {
    private int iterator = 0;

    // Poses
    private final PoseBuilder pb = new PoseBuilder(Distance.Units.INCHES, Angle.Units.DEGREES, false);
    final Pose[] poses = {
            pb.build(0, 0, 0), // startPose
            //pb.build(24, 0, 0), // X movement only
            //pb.build(0, 24, 0), // Y movement only
            pb.build(0, 0, 180), // Heading movement only
            //pb.build(24, 24, 0) // Translational only
            //pb.build(24, 24, 90) // All at once
    };

    @Override
    public void runOpMode() {
        P2PFollower follower = (P2PFollower) new Constants().build(hardwareMap, Pose.zero());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            if (!follower.isBusy()) {
                if (iterator < poses.length - 1) {
                    iterator++;
                    follower.setTargetPose(poses[iterator]);
                } else {
                    // We've reached the final pose
                    follower.stop();
                    telemetry.addData("Status", "Done");
                }
            }

            if (gamepad1.a) { // Emergency stop
                follower.stop();
                telemetry.addData("Status", "Stopped");
            }

            telemetry.addData("Current Pose", follower.getPose().toString());
            telemetry.addData("Target Pose", follower.getTargetPose().toString());
            telemetry.addData("Velocity", follower.getVelocity().toString());
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.addData("Translational at target", follower.translationalAtTarget());
            telemetry.addData("Heading at target", follower.headingAtTarget());
            telemetry.update();
        }
    }
}
