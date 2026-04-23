package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;

import commands.CommandScheduler;
import commands.InstantCommand;

import followers.quintic.PathBuilder;
import followers.quintic.QuinticFollower;
import util.Pose;

/**
 * Test auto for quintic splines
 *
 * @author Sohum Arora 22985 Paraducks
 */
@Autonomous(name = "Apex Quintic Test Auto", group = "Apex Pathing Tests")
public class QuinticTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // NOTE: This won't work until we make a quintic constants class and add it to the Constants builder
        QuinticFollower follower = (QuinticFollower) new Constants().build(hardwareMap, Pose.zero());

        CommandScheduler.getInstance().schedule(
                new InstantCommand(()->
                    new PathBuilder()
                        .addPose(0, 0, 0)
                        .addPose(24, 24, 90)
                        .addPose(48, 0, 0)
                        .holdAtPathEnd(500)
                        .build()
                ));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            CommandScheduler.getInstance().run();
        }
    }
}