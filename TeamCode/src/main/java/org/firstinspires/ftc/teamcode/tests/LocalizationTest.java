package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import followers.P2PFollower;
import util.Pose;

/**
 * Test OpMode for using Apex Pathing in TeleOp mode.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@SuppressWarnings("unused")
@TeleOp(name = "Apex LocalizationTest", group = "Apex Pathing Tests")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        P2PFollower follower = (P2PFollower) new Constants().build(hardwareMap, Pose.zero());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();

            if (gamepad1.left_trigger_pressed) { // Emergency stop
                follower.stop();
                telemetry.addLine("Follower stopped");
            } else {
                follower.drive(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        currentPose.getHeading() // (Can be removed if you never use field-centric)
                );
            }

            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y ",currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }
    }
}