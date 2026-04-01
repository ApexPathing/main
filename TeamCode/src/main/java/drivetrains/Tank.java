package Drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import Drivetrains.Constants.TankConstants;
import motors.MotorEx;

@SuppressWarnings("unused")
public class Tank extends Drivetrain {
    TankConstants constants;
    ArrayList<MotorEx> left = new ArrayList<>();
    ArrayList<MotorEx> right = new ArrayList<>();

    public Tank(HardwareMap hardwareMap, @NonNull TankConstants constants) {
        this.constants = constants;

        left.add(new MotorEx(hardwareMap, constants.leftFrontData));
        right.add(new MotorEx(hardwareMap, constants.rightFrontData));


        if (constants.fourMotorTankDrive) {
            left.add(new MotorEx(hardwareMap, constants.leftRearData));
            right.add(new MotorEx(hardwareMap, constants.rightRearData));
        }
    }
    @Override
    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (MotorEx motor : left) {
            motor.motor.setZeroPowerBehavior(behavior);
        }
        for (MotorEx motor : right) {
            motor.motor.setZeroPowerBehavior(behavior);
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
        //TODO: Implement field-centric drive
        moveWithVectors(y, x, turn);

    }

    @Override
    public void stop() {

    }

    @Override
    public void debug(Telemetry telemetry) {

    }

    public void setPowers(double leftPower, double rightPower) {
        for (MotorEx motor : left) {
            motor.motor.setPower(leftPower);
        }
        for (MotorEx motor : right) {
            motor.motor.setPower(rightPower);
        }
    }
}
