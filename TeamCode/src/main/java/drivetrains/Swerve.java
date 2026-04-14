package drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import drivetrains.constants.SwerveConstants;
import motors.MotorEx;

/**
 * @author Xander Haemel
 */
public class Swerve extends Drivetrain{
    SwerveConstants constants;

    // Motors
    MotorEx flMotor;
    MotorEx blMotor;
    MotorEx frMotor;
    MotorEx brMotor;


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

    }

    /**
     * set power based on inputs
     * @param lfPower is left front power from 0.0 - 0.1
     * @param lrPower is left rear power from 0.0 - 0.1
     * @param rfPower is right front power from 0.0 - 0.1
     * @param rrPower is right rear power from 0.0 - 0.1
     */
    private void setPower(double lfPower, double lrPower, double rfPower, double rrPower){
        double max = Math.max(0, Math.abs(lfPower));
        max = Math.max(max, Math.abs(lrPower));
        max = Math.max(max, Math.abs(rfPower));
        max = Math.max(max, Math.abs(rrPower));
        if (max > constants.MotorMaxPower) {
            lfPower = (lfPower / max) * constants.MotorMaxPower;
            lrPower = (lrPower / max) * constants.MotorMaxPower;
            rfPower = (rfPower / max) * constants.MotorMaxPower;
            rrPower = (rrPower / max) * constants.MotorMaxPower;
        }
        flMotor.motor.setPower(lfPower);
        blMotor.motor.setPower(rrPower);
        frMotor.motor.setPower(rfPower);
        flMotor.motor.setPower(rrPower);
    }

    @Override
    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {

    }

    @Override
    public void moveWithVectors(double drive, double strafe, double turn) {

    }

    @Override
    public void drive(double x, double y, double turn, double robotHeading) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void debug(Telemetry telemetry) {
        telemetry.addData("Front Left Power", flMotor.motor.getPower());
        telemetry.addData("Front Right Power", frMotor.motor.getPower());
        telemetry.addData("Back left Power", blMotor.motor.getPower());
        telemetry.addData("Back Right Power", brMotor.motor.getPower());
    }
}
