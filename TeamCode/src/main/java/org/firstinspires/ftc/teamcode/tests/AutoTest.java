package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

import followers.P2PFollower;
import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseFactory;

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
    private final PoseFactory pb = new PoseFactory(Distance.Units.INCHES, Angle.Units.DEGREES, false);
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

            if (!follower.isBusy() || timerStarted) {
                if (iterator < poses.length - 1) {
                    if (!timerStarted) {
                        waitTimer.reset();
                        timerStarted = true;
                        follower.holdPose(poses[iterator]);
                    } else if (waitTimer.milliseconds() >= 2000) {
                        timerStarted = false;
                        iterator++;
                        follower.setTargetPose(poses[iterator]);
                    }
                } else {
                    follower.holdPose(poses[poses.length - 1]);
                }
            }

            if (gamepad1.a) { // Emergency stop
                follower.stop();
                telemetry.addData("Status", "Stopped");
            } else if (iterator >= poses.length - 1) {
                telemetry.addData("Status", "Done");
            } else {
                telemetry.addData("Status", "Moving to Pose " + (iterator + 1));
            }

            if (follower.getTargetPose() != null) {
                telemetry.addData("Target Pose", follower.getTargetPose().toString());
            }

            telemetry.addData("Current Pose", follower.getPose().toString());
            //telemetry.addData("Velocity", follower.getVelocity().toString());
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.addData("Axial at target", follower.axialAtTarget());
            telemetry.addData("Strafe at target", follower.strafeAtTarget());
            telemetry.addData("Heading at target", follower.headingAtTarget());
            telemetry.addData("Holding Pose", follower.isHoldingPose());
            telemetry.update();
        }
    }
}
