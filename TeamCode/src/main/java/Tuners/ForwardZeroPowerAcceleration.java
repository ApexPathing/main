package Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;

import drivetrains.Mecanum;
import followers.P2PFollower;
import localizers.Pinpoint;
import util.Angle;
import util.Distance;
import util.Pose;

public class ForwardZeroPowerAcceleration extends LinearOpMode {
    Pose startPose = new Pose(0, 0, 0, Distance.Units.INCHES, Angle.Units.DEGREES, false);
    //distance to travel
    public double distance = 30; //inches
    public double topSpeed = 0; //inches per second
    public boolean positionReached;

    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        Pinpoint localizer = new Pinpoint(hardwareMap, Constants.localizerConstants, startPose);
        P2PFollower follower = new P2PFollower(Constants.followerConstants, drivetrain, localizer);

        waitForStart();
        while(opModeIsActive()){

        }





    }
}
