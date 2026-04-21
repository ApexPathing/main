package drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.SwerveModule;
import drivetrains.constants.SwerveConstants;
import hardware.MotorEx;
import util.SwerveUnit;

/**
 * swerve drive class
 * @author Xander Haemel
 */
public class Swerve extends Drivetrain{
    SwerveConstants constants;

    // Motors
    MotorEx flMotor;
    MotorEx blMotor;
    MotorEx frMotor;
    MotorEx brMotor;

    SwerveModule fl;
    SwerveModule rl;
    SwerveModule fr;
    SwerveModule rr;



    /**
     * default constructor
     * @param hardwareMap is the hardwaremap
     * @param constants: swerveconstants, containing the configuration for your drivetrain
     */
    public Swerve(HardwareMap hardwareMap, @NonNull SwerveConstants constants){
        this.constants = constants;
        //motors
        flMotor = new MotorEx(hardwareMap, constants.flData);
        frMotor = new MotorEx(hardwareMap, constants.frData);
        blMotor = new MotorEx(hardwareMap, constants.blData);
        brMotor = new MotorEx(hardwareMap, constants.brData);
        //new swerve modules
        fl = new SwerveModule(hardwareMap, flMotor);
        rl = new SwerveModule(hardwareMap, blMotor);
        fr = new SwerveModule(hardwareMap, frMotor);
        rr = new SwerveModule(hardwareMap, brMotor);
    }



    @Override
    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {

    }

    /**
     * moves the robot with vectors
     * @param drive the forward/backward movement vector (positive for forward, negative for backward)
     * @param strafe the left/right movement vector (positive for right, negative for left)
     * @param turn the rotation vector (positive for clockwise, negative for counterclockwise)
     */
    @Override
    public void moveWithVectors(double drive, double strafe, double turn){
        //make turn angle clockwise
        turn /= -1;
        //used for the sideways motion of both rear wheels
        double strafeRear = strafe - turn * (constants.wheelbase/constants.diagonalDistance);
        //used for the sideways motion of both front wheels
        double strafeFront  = strafe + turn * (constants.wheelbase/constants.diagonalDistance);
        //used for the forward motion of the right wheels
        double forwardRight = drive  - turn * (constants.trackwidth/constants.diagonalDistance);
        //used for the forward motion of the left wheels
        double forwardLeft  = drive  + turn * (constants.trackwidth/constants.diagonalDistance);

        //now we calculate wheel speeds based on these variables
        double frontRightSpeed = Math.sqrt(Math.pow(strafeFront,2) + Math.pow(forwardRight, 2));
        double frontLeftSpeed = Math.sqrt(Math.pow(strafeFront,2) + Math.pow(forwardLeft, 2));
        double rearLeftSpeed = Math.sqrt(Math.pow(strafeRear,2) + Math.pow(forwardLeft, 2));
        double rearRightSpeed = Math.sqrt(Math.pow(strafeRear,2) + Math.pow(forwardRight, 2));

        //optimize and calculate wheel angles rather than turning 180 degrees
        SwerveUnit frontRight = optimizeWheelAngle(fr.getPodHeading(), Math.atan2(strafeFront, forwardRight)*180/Math.PI, frontRightSpeed);
        SwerveUnit frontLeft  = optimizeWheelAngle(fl.getPodHeading(), Math.atan2(strafeFront, forwardLeft)*180/Math.PI,  frontLeftSpeed);
        SwerveUnit rearLeft   = optimizeWheelAngle(rl.getPodHeading(), Math.atan2(strafeRear, forwardLeft)*180/Math.PI, rearLeftSpeed);
        SwerveUnit rearRight  = optimizeWheelAngle(rr.getPodHeading(), Math.atan2(strafeRear,forwardRight)*180/Math.PI, rearRightSpeed);

        //scale powers to be =<1
        double max = Math.abs(frontRight.getMotorPower());
        if(Math.abs(frontLeft.getMotorPower())> max){
            max = Math.abs(frontLeft.getMotorPower());
        } if(Math.abs(rearLeft.getMotorPower()) > max){
            max = Math.abs(rearLeft.getMotorPower());
        } if(Math.abs(rearRight.getMotorPower())> max){
            max = Math.abs(rearRight.getMotorPower());
        }
        //scale down if powers are greater than 1
        if(max > 1){
            frontRight.setMotorSpeed(frontRight.getMotorPower() / max);
            frontLeft.setMotorSpeed(frontLeft.getMotorPower() / max);
            rearLeft.setMotorSpeed(rearLeft.getMotorPower() / max);
            rearRight.setMotorSpeed(rearRight.getMotorPower() / max);
        }
        //setPowersAndAngles
        fr.setPodAngleAndPower(frontRight);
        fl.setPodAngleAndPower(frontLeft);
        rl.setPodAngleAndPower(rearLeft);
        rr.setPodAngleAndPower(rearRight);
    }

    /**
     * optimizes wheel angle by flipping the motor direction when possible
     * @param currentAngle is the angle of the current pod (degrees)
     * @param targetAngle is the target angle for the pod (degrees)
     * @param power the power of the swerve pod 0.0-1.0
     * @return the pod angle (degrees)
     */
    private SwerveUnit optimizeWheelAngle(double currentAngle, double targetAngle, double power){
        double delta = targetAngle- currentAngle;
        double wrappedDelta = delta - (360 * Math.round(delta /360.0));
        if(Math.abs(wrappedDelta) > 90){
            power *= -1;
            wrappedDelta -= Math.copySign(180, wrappedDelta);
        }
        return new SwerveUnit(power, currentAngle + wrappedDelta);
    }

    @Override
    public void drive(double x, double y, double turn, double robotHeading) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void debug(Telemetry telemetry) {

    }

    /**
     * call this every loop
     */
    public void update(){
        fl.update();
        fr.update();
        rl.update();
        rr.update();


    }
}
