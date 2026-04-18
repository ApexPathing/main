package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import followers.P2PFollower;
import drivetrains.Mecanum;
import localizers.OTOS;
import localizers.Pinpoint;
import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseBuilder;

@Autonomous(name = "Mecanum Test Auto", group = "Apex Pathing Tests")
public class MecanumAuto extends LinearOpMode {
    private int iterator = 0;

    // Poses
    private final PoseBuilder pb = new PoseBuilder(Distance.Units.INCHES, Angle.Units.DEGREES, false);
    final Pose[] poses = {
            pb.build(0, 0, 0), // startPose
            pb.build(0, 0, 180), // X movement
            //pb.build(24, 24, 0), // Y movement
            //pb.build(24, 24, 90), // Heading movement
            //pb.build(0, 0, 0) // All at once
    };

    @Override
    public void runOpMode() {
        // !!!! NOTE: Do not directly use the drivetrain or localizer in the opmode, only use the follower !!!!
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        OTOS localizer = new OTOS(hardwareMap, Constants.localizerConstants, poses[0]);
        P2PFollower follower = new P2PFollower(Constants.followerConstants, drivetrain, localizer);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!follower.isBusy()) {
                if (iterator < poses.length - 1) {
                    iterator++;
                    follower.setTargetPose(poses[iterator]);
                } else {
                    // We've reached the final pose
                    follower.stop();
                    requestOpModeStop();
                }
            }

            follower.update();

            // Stop
            if (gamepad1.a) {
                requestOpModeStop();
            }

            telemetry.addData("Auto State", iterator);
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
