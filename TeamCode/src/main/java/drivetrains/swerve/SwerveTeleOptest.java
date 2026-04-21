package drivetrains.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Constants;

import drivetrains.constants.SwerveConstants;
import followers.P2PFollower;
import localizers.Pinpoint;
import localizers.constants.PinpointConstants;
import util.Pose;

/**
 * TeleOp for the Swerve drivetrain using a P2PFollower and Pinpoint localizer.
 * @author Sohum Arora - 22985 Paraducks
 */

@TeleOp(name = "Swerve TeleOp", group = "Apex Pathing Tests")
public class SwerveTeleOptest extends LinearOpMode {
    PinpointConstants constants;

    @Override
    public void runOpMode() {
        Swerve drivetrain = new Swerve(hardwareMap, new SwerveConstants());
        Pinpoint localizer = new Pinpoint(hardwareMap,constants , Pose.zero());
        P2PFollower follower = new P2PFollower(Constants.followerConstants, drivetrain, localizer);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            localizer.update();
            Pose currentPose = localizer.getPose();

            double x =  gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn =gamepad1.right_stick_x;

            if (gamepad1.left_trigger_pressed) {
                follower.stop();
                telemetry.addLine("Follower stopped");
            } else {
                follower.drive(x, y, turn, currentPose.getHeading());
            }

            drivetrain.update();

            telemetry.addData("Pose", currentPose.toString());
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            drivetrain.logData(telemetry);
            telemetry.update();
        }
    }
}