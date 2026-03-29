package Tests;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Drivetrains.Constants.MecanumConstants;

public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .setLeftFrontMotorName("leftFront")
            .setLeftRearMotorName("leftRear")
            .setRightFrontMotorName("rightFront")
            .setRightRearMotorName("rightRear")
            .setLeftFrontDirection(DcMotorSimple.Direction.FORWARD)
            .setLeftRearDirection(DcMotorSimple.Direction.FORWARD)
            .setRightFrontDirection(DcMotorSimple.Direction.REVERSE)
            .setRightRearDirection(DcMotorSimple.Direction.REVERSE)
            .setUseBrakingMode(true)
            .setRobotCentric(true)
            .setMaxPower(0.5);
}
