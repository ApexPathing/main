package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

import followers.P2PFollower;
import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseBuilder;

/**
 * Test OpMode for using Apex Pathing in Autonomous mode.
 * Edit the poses array to test different types of movement individually or together.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Autonomous(name = "Apex Autonomous Test", group = "Apex Pathing Tests")
public class AutoTest extends LinearOpMode {
    private int iterator = 0;

    private final ElapsedTime waitTimer = new ElapsedTime();
    private boolean timerStarted = true; // Start with true so it doesn't wait to move to the first pose

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
                    // Wait before moving on for the robot to settle
                    if (!timerStarted) {
                        waitTimer.reset();
                        timerStarted = true;
                    } else if (waitTimer.milliseconds() >= 1000) {
                        timerStarted = false;
                        iterator++;
                        follower.setTargetPose(poses[iterator]);
                    } else {
                        continue; // Wait until the timer has elapsed
                    }
                } else {
                    // We've reached the final pose
                    follower.stop(); // Hold position
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
            telemetry.addData("Axial at target", follower.axialAtTarget());
            telemetry.addData("Strafe at target", follower.strafeAtTarget());
            telemetry.addData("Heading at target", follower.headingAtTarget());
            telemetry.update();
        }
    }
}
