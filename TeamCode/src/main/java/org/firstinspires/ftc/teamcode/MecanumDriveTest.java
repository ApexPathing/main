package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import drivetrains.Mecanum;
import followers.P2PFollower;
import localizers.Pinpoint;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * Test for the Mecanum drivetrain class using a Pinpoint localizer
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@SuppressWarnings("unused")
@TeleOp(name = "MecanumDrive Test", group = "Apex beta test")
public class MecanumDriveTest extends LinearOpMode {
    Pose startPose = new Pose(0, 0, 0, Distance.Units.INCHES, Angle.Units.DEGREES, false);

    @Override
    public void runOpMode() {
        // !!!! NOTE: Do not directly use the drivetrain or localizer in the opmode, only use the follower !!!!
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        Pinpoint localizer = new Pinpoint(hardwareMap, Constants.localizerConstants, startPose);
        P2PFollower follower = new P2PFollower(Constants.followerConstants, drivetrain, localizer);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Update localizer and grab current pose
            localizer.update();
            Pose currentPose = localizer.getPose();

            // Read driver inputs (invert Y so forward stick = positive drive)
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Start: emergency stop — cuts all motor power
            if (gamepad1.start) {
                follower.stop();
                telemetry.addLine("Follower stopped");
            } else {
                follower.drive(x, y, turn, currentPose.getHeading());
            }

            // Telemetry output
            telemetry.addData("Pose", currentPose.toString());
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y ",currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }
    }
}