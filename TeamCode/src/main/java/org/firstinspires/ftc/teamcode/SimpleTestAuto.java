package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import followers.P2PFollower;
import drivetrains.Mecanum;
import localizers.Pinpoint;
import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseBuilder;

@Autonomous(name = "SimpleTestAuto", group = "tests")
public class SimpleTestAuto extends LinearOpMode {
    private P2PFollower follower;
    private int iterator = 0;

    // Poses
    // pb is for peanut butter (～￣▽￣)～
    private final PoseBuilder pb = new PoseBuilder(Distance.Units.INCHES, Angle.Units.DEGREES, false);
    final Pose[] poses = {
        pb.build(0, 0, 0), // startPose
        pb.build(24, 0, 0), // test X movement
        pb.build(24, 24, 0), // Test Y movement
        pb.build(24, 24, 90), // Test rotation
        pb.build(0, 24, 90), // Test rotation + x movement
        pb.build(0, 0, 90) // Test rotation + y movement
    };


    @Override
    public void runOpMode() throws InterruptedException {
        // !!!! NOTE: Do not directly use the drivetrain or localizer in the opmode, only use the follower !!!!
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        Pinpoint localizer = new Pinpoint(hardwareMap, Constants.localizerConstants, poses[0]);
        follower = new P2PFollower(Constants.followerConstants, drivetrain, localizer);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();

            if (!follower.isBusy() && iterator < poses.length - 1) {
                iterator ++;
                follower.setTargetPose(poses[iterator]);
            } else {
                follower.stop();
            }

            // Stop
            if (gamepad1.a) {
                requestOpModeStop();
            }

            telemetry.addData("Auto State", iterator);
            telemetry.addData("Current Pose", follower.getPose().toString());
            telemetry.addData("Target Pose", follower.getTargetPose().toString());
            telemetry.addData("Velocity", follower.getVelocity().toString());
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.update();
        }
    }
}
