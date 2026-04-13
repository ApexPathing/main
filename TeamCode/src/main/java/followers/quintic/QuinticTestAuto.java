package followers.quintic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import commands.CommandScheduler;
import commands.InstantCommand;
import drivetrains.Mecanum;

import localizers.Pinpoint;
import org.firstinspires.ftc.teamcode.Constants;
import util.Pose;

/**
 * Test auto for quintic
 * @author Sohum Arora 22985 Paraducks
 */

@Autonomous(name = "Quintic Test Auto", group = "Apex Test")
public class QuinticTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose startPose = new Pose(0, 0, 0);

        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        Pinpoint localizer = new Pinpoint(hardwareMap, Constants.localizerConstants, startPose);
        QuinticFollower follower = new QuinticFollower(drivetrain, localizer);

        waitForStart();

        CommandScheduler.getInstance().schedule(
                new InstantCommand(()->
                    new PathBuilder()
                        .addPose(0, 0, 0)
                        .addPose(24, 24, 90)
                        .addPose(48, 0, 0)
                        .holdAtPathEnd(500)
                        .build()
                ));

        while (opModeIsActive()) {
            localizer.update();
            CommandScheduler.getInstance().run();
        }
    }
}