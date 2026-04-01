package tests;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import drivetrains.constants.MecanumConstants;
import localizers.constants.PinpointConstants;
import followers.constants.P2PFollowerConstants;

/**
 * Constants file for testing
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
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

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .setName("pinpoint")
            .setDistanceUnit(DistanceUnit.INCH)
            .setAngleUnit(AngleUnit.DEGREES)
            .setXOffset(-3.31) // In distanceUnit // TODO: Auto offset tuner
            .setYOffset(-6.61) // In distanceUnit // TODO: Auto offset tuner
            .setXPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .setYPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    public static P2PFollowerConstants followerConstants = new P2PFollowerConstants()
            .setTranslationalKp(0.03) // TODO: Tuner
            .setHeadingKp(0.5) // TODO: Tuner
            .setTranslationalTolerance(1.0) // Inches
            .setHeadingTolerance(3.0) // Degrees
            .setMaxPower(0.5) // Power limits can be overwritten by the drivetrain's power limits, these are specifically for following
            .setMinPower(0.05);
}