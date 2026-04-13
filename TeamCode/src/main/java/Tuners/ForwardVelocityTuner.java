package Tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;

import drivetrains.Mecanum;
import followers.P2PFollower;
import localizers.Pinpoint;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * A tester for forward velocity of robot
 * @author Xander Haemel - 31616
 */
@Autonomous
public class ForwardVelocityTuner extends LinearOpMode {
    Pose startPose = new Pose(0, 0, 0, Distance.Units.INCHES, Angle.Units.DEGREES, false);
    //distance to travel
    public double distance = 40; //inches
    public double topSpeed = 0; //inches per second
    public boolean positionReached;

    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        Pinpoint localizer = new Pinpoint(hardwareMap, Constants.localizerConstants, startPose);
        P2PFollower follower = new P2PFollower(Constants.followerConstants, drivetrain, localizer);
        waitForStart();
        while (opModeIsActive()){
            if(follower.getPose().getX() > distance){
                positionReached = true;
                follower.drive(0,0,0);
                telemetry.addData("Forward Velocity:", topSpeed);
                telemetry.update();
            }else{
                positionReached = false;
                follower.drive(1, 0, 0);
            }
            //main loop
            if(!positionReached){
                Pose robotVel = follower.getVelocity();
                //telemetry
                telemetry.addData("current Velocity", robotVel.getX());
                telemetry.update();

                //record speed
                if(robotVel.getX() > topSpeed){
                    topSpeed = robotVel.getX();
                }
            }
            //e-stop
            if(gamepad1.a){
                terminateOpModeNow();
            }

        }
    }
}
