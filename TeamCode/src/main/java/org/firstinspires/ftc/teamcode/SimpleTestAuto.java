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
    public enum AutoState {
        ROTATE, FORWARD, END
    }
    public AutoState autoState = AutoState.ROTATE;

    // Poses
    PoseBuilder pb = new PoseBuilder(Distance.Units.INCHES, Angle.Units.DEGREES, false);
    Pose startPose = pb.build(0, 0, 0);
    Pose turnedPose = pb.build(0, 0, 90);
    Pose forwardPose = pb.build(24, 0, 90);

    @Override
    public void runOpMode() throws InterruptedException {
        // !!!! NOTE: Do not directly use the drivetrain or localizer in the opmode, only use the follower !!!!
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        Pinpoint localizer = new Pinpoint(hardwareMap, Constants.localizerConstants, startPose);
        P2PFollower follower = new P2PFollower(Constants.followerConstants, drivetrain, localizer);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();

            switch(autoState) {
                case ROTATE:
                    follower.setTargetPose(turnedPose);
                    if (!follower.isBusy()) {
                        autoState = AutoState.FORWARD;
                    }
                    break;
                case FORWARD:
                    follower.setTargetPose(forwardPose);
                    if (!follower.isBusy()) {
                        autoState = AutoState.END;
                    }
                    break;
                case END:
                    follower.stop();
                    break;
            }

            // Stop
            if (gamepad1.a) {
                requestOpModeStop();
            }

            telemetry.addData("Auto State", autoState);
            telemetry.addData("Current Pose", follower.getPose().toString());
            telemetry.addData("Target Pose", follower.getTargetPose().toString());
            telemetry.addData("Velocity", follower.getVelocity().toString());
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.update();
        }
    }
}
