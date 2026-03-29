package Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Followers.P2PFollower;
import Drivetrains.Mecanum;
import Localizers.Pinpoint;
import Util.Pose;

@Autonomous(name = "SimpleTestAuto", group = "Tests")
public class SimpleTestAuto extends LinearOpMode {
    public enum AutoState {
        ROTATE, FORWARD, END
    }
    public AutoState autoState = AutoState.ROTATE;

    // Poses
    Pose startPose = new Pose(0, 0, 0);
    Pose turnedPose = new Pose(0, 0, Math.toRadians(90));
    Pose forwardPose = new Pose(24, 0, Math.toRadians(90));

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
            telemetry.addData("Current Pose", follower.getPose().debug());
            telemetry.addData("Target Pose", follower.getTargetPose().debug());
            telemetry.addData("Velocity", follower.getVelocity().toString());
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.update();
        }
    }
}
