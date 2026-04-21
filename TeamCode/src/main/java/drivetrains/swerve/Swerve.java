package drivetrains.Swerve;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import drivetrains.Drivetrain;
import drivetrains.constants.SwerveConstants;
import hardware.MotorEx;

/**
 * Swerve drive class.
 * Call the update method every loop
 * Use moveWithVectors method for teleop.
 * @author Xander Haemel - 31616 404 Not Found
 * @author Sohum Arora - 22985 Paraducks
 */
public class Swerve extends Drivetrain {

    SwerveConstants constants;

    MotorEx flMotor, frMotor, blMotor, brMotor;
    SwerveModule fl, fr, rl, rr;

    /**
     * @param hardwareMap hardware map from OpMode
     * @param constants   SwerveConstants with your robot's config
     */
    public Swerve(HardwareMap hardwareMap, @NonNull SwerveConstants constants) {
        this.constants = constants;

        // Init motors
        flMotor = new MotorEx(hardwareMap, constants.flData);
        frMotor = new MotorEx(hardwareMap, constants.frData);
        blMotor = new MotorEx(hardwareMap, constants.blData);
        brMotor = new MotorEx(hardwareMap, constants.brData);

        // Init swerve modules (motor + servo + encoder)
        fl = new SwerveModule(hardwareMap, flMotor, constants.flServo, constants.flEncoder);
        fr = new SwerveModule(hardwareMap, frMotor, constants.frServo, constants.frEncoder);
        rl = new SwerveModule(hardwareMap, blMotor, constants.blServo, constants.blEncoder);
        rr = new SwerveModule(hardwareMap, brMotor, constants.brServo, constants.brEncoder);
    }

    /**
     * Field-centric drive. Rotates input vectors by robot heading before passing to moveWithVectors.
     *
     * @param x strafe input
     * @param y drive input
     * @param turn turn input
     * @param robotHeading current robot heading in radians
     */
    @Override
    public void drive(double x, double y, double turn, double robotHeading) {
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double fieldX = x * cos - y * sin;
        double fieldY = x * sin + y * cos;
        moveWithVectors(fieldY, fieldX, turn);
    }

    /**
     * Robot-centric swerve drive.
     *
     * @param drive  forward/back  [-1, 1], positive = forward
     * @param strafe left/right    [-1, 1], positive = right
     * @param turn   rotation      [-1, 1], positive = clockwise
     */
    @Override
    public void moveWithVectors(double drive, double strafe, double turn) {
        turn /= -1.0;

        double strafeRear  = strafe - turn * (constants.wheelbase  / constants.diagonalDistance);
        double strafeFront = strafe + turn * (constants.wheelbase  / constants.diagonalDistance);
        double forwardRight = drive - turn * (constants.trackwidth / constants.diagonalDistance);
        double forwardLeft  = drive + turn * (constants.trackwidth / constants.diagonalDistance);

        double frSpeed = Math.sqrt(strafeFront * strafeFront + forwardRight * forwardRight);
        double flSpeed = Math.sqrt(strafeFront * strafeFront + forwardLeft  * forwardLeft);
        double rlSpeed = Math.sqrt(strafeRear  * strafeRear  + forwardLeft  * forwardLeft);
        double rrSpeed = Math.sqrt(strafeRear  * strafeRear  + forwardRight * forwardRight);

        SwerveUnit frontRight = optimizeWheelAngle(fr.getPodHeading(), Math.toDegrees(Math.atan2(strafeFront, forwardRight)), frSpeed);
        SwerveUnit frontLeft  = optimizeWheelAngle(fl.getPodHeading(), Math.toDegrees(Math.atan2(strafeFront, forwardLeft)),  flSpeed);
        SwerveUnit rearLeft = optimizeWheelAngle(rl.getPodHeading(), Math.toDegrees(Math.atan2(strafeRear,  forwardLeft)),  rlSpeed);
        SwerveUnit rearRight = optimizeWheelAngle(rr.getPodHeading(), Math.toDegrees(Math.atan2(strafeRear,  forwardRight)), rrSpeed);

        // Scale powers down if any exceed 1.0
        double max = Math.max(Math.max(Math.abs(frontRight.getMotorPower()), Math.abs(frontLeft.getMotorPower())), Math.max(Math.abs(rearLeft.getMotorPower()), Math.abs(rearRight.getMotorPower())));
        if (max > 1.0) {
            frontRight.setMotorSpeed(frontRight.getMotorPower() / max);
            frontLeft.setMotorSpeed (frontLeft.getMotorPower()  / max);
            rearLeft.setMotorSpeed  (rearLeft.getMotorPower()   / max);
            rearRight.setMotorSpeed (rearRight.getMotorPower()  / max);
        }

        fr.setPodAngleAndPower(frontRight);
        fl.setPodAngleAndPower(frontLeft);
        rl.setPodAngleAndPower(rearLeft);
        rr.setPodAngleAndPower(rearRight);
    }

    /**
     * Optimizes wheel angle: flip motor direction instead of rotating > 90 degrees.
     */
    private SwerveUnit optimizeWheelAngle(double currentAngle, double targetAngle, double power) {
        double delta = targetAngle - currentAngle;
        double wrappedDelta = delta - (360.0 * Math.round(delta / 360.0));
        if (Math.abs(wrappedDelta) > 90) {
            power *= -1;
            wrappedDelta -= Math.copySign(180, wrappedDelta);
        }
         return new SwerveUnit(power, currentAngle + wrappedDelta);
    }

    /** Call every loop iteration. */
    public void update() {
        fl.update();
        fr.update();
        rl.update();
        rr.update();
    }

    @Override
    public void stop() {
        moveWithVectors(0, 0, 0);
        update();
    }

    @Override
    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        flMotor.setBrakeMode(behavior);
        frMotor.setBrakeMode(behavior);
        blMotor.setBrakeMode(behavior);
        brMotor.setBrakeMode(behavior);
    }

    @Override
    public void logData(Telemetry telemetry) {
        telemetry.addData("FL heading", fl.getPodHeading());
        telemetry.addData("FR heading", fr.getPodHeading());
        telemetry.addData("RL heading", rl.getPodHeading());
        telemetry.addData("RR heading", rr.getPodHeading());
        telemetry.addData("FL power", flMotor.getPower());
        telemetry.addData("FR power", frMotor.getPower());
        telemetry.addData("BL power", blMotor.getPower());
        telemetry.addData("BR power", brMotor.getPower());
    }
}