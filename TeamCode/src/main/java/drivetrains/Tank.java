package drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import drivetrains.constants.TankConstants;
import hardware.MotorEx;

/**
 * Tank Drivetrain controller class
 * @author Xander Haemel - 31616 - 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Tank extends Drivetrain {
    TankConstants constants;

    // Motors
    MotorEx flMotor;
    MotorEx blMotor; // Only used for 4 motor tank drive
    MotorEx frMotor;
    MotorEx brMotor; // Only used for 4 motor tank drive

    public Tank(HardwareMap hardwareMap, @NonNull TankConstants constants) {
        this.constants = constants;

        flMotor = new MotorEx(hardwareMap, constants.flData);
        frMotor = new MotorEx(hardwareMap, constants.frData);

        if (constants.fourMotor) {
            blMotor = new MotorEx(hardwareMap, constants.blData);
            brMotor = new MotorEx(hardwareMap, constants.brData);
        }
    }

    @Override
    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        flMotor.setBrakeMode(behavior);
        frMotor.setBrakeMode(behavior);

        if (constants.fourMotor) {
            blMotor.setBrakeMode(behavior);
            brMotor.setBrakeMode(behavior);
        }
    }

    @Override
    public void moveWithVectors(double drive, double strafe, double turn) {
        // Interpreting the values
        double leftSidePower = drive + turn;
        double rightSidePower = drive - turn;

        // Normalizing the above values to ensure we are passing valid values to the motors
        double max = Math.max(Math.abs(leftSidePower), Math.abs(rightSidePower));
        if (max > 1.0) {
            leftSidePower /= max;
            rightSidePower /= max;
        }

        // Finally, simply passing those velocities to the motors, which will take them
        setPowers(leftSidePower, rightSidePower);
    }

    @Override
    public void drive(double x, double y, double turn, double robotHeading) {
        moveWithVectors(y, x, turn);
    }

    public void setPowers(double leftPower, double rightPower) {
        flMotor.motor.setPower(leftPower);
        frMotor.motor.setPower(rightPower);

        if (constants.fourMotor) {
            blMotor.motor.setPower(leftPower);
            brMotor.motor.setPower(rightPower);
        }
    }

    @Override
    public void stop() {
        setPowers(0, 0);
    }

    @Override
    public void logData(Telemetry telemetry) {
        telemetry.addData("Front Left Power", flMotor.motor.getPower());
        telemetry.addData("Front Right Power", frMotor.motor.getPower());

        if (constants.fourMotor) {
            telemetry.addData("Back left Power", blMotor.motor.getPower());
            telemetry.addData("Back Right Power", brMotor.motor.getPower());
        }
    }
}
