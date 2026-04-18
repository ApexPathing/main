package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import drivetrains.Mecanum;
import followers.P2PFollower;
import localizers.OTOS;
import localizers.Pinpoint;
import util.Pose;

/**
 * Test for the Mecanum drivetrain class using a Pinpoint localizer
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@SuppressWarnings("unused")
@TeleOp(name = "Mecanum TeleOp", group = "Apex Pathing Tests")
public class MecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // !!!! NOTE: Do not directly use the drivetrain or localizer in the opmode, only use the follower !!!!
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        OTOS localizer = new OTOS(hardwareMap, Constants.localizerConstants, Pose.zero());
        P2PFollower follower = new P2PFollower(Constants.followerConstants, drivetrain, localizer);
        Servo testServo = hardwareMap.get(Servo.class, "servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Update localizer and grab current pose
            localizer.update();
            Pose currentPose = localizer.getPose();

            // Invert Y and turn inputs
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if (gamepad1.left_trigger_pressed) { // Emergency stop
                follower.stop();
                telemetry.addLine("Follower stopped");
            } else {
                follower.drive(x, y, turn, currentPose.getHeading());
            }

            if (gamepad1.a) {
                testServo.setPosition(0);
            } else if (gamepad1.b) {
                testServo.setPosition(1);
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