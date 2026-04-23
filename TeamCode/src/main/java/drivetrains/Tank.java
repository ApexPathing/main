package drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import drivetrains.constants.TankConstants;

/**
 * Tank Drivetrain controller class
 *
 * @author Xander Haemel - 31616 - 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Tank extends Drivetrain {
    TankConstants constants;

    // Motors
    DcMotorEx flMotor;
    DcMotorEx blMotor; // Only used for 4 motor tank drive
    DcMotorEx frMotor;
    DcMotorEx brMotor; // Only used for 4 motor tank drive

    public Tank(HardwareMap hardwareMap, @NonNull TankConstants constants) {
        this.constants = constants;

        flMotor = this.constants.flData.build(hardwareMap);
        frMotor = this.constants.frData.build(hardwareMap);

        if (constants.fourMotor) {
            blMotor = this.constants.blData.build(hardwareMap);
            brMotor = this.constants.brData.build(hardwareMap);
        }
    }

    @Override
    protected void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        flMotor.setZeroPowerBehavior(behavior);
        frMotor.setZeroPowerBehavior(behavior);

        if (constants.fourMotor) {
            blMotor.setZeroPowerBehavior(behavior);
            brMotor.setZeroPowerBehavior(behavior);
        }
    }

    @Override
    protected boolean isRobotCentric() {
        return this.constants.robotCentric;
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
        flMotor.setPower(leftPower);
        frMotor.setPower(rightPower);

        if (constants.fourMotor) {
            blMotor.setPower(leftPower);
            brMotor.setPower(rightPower);
        }
    }

    @Override
    public void stop() {
        setPowers(0, 0);
    }

    @Override
    public void debug(Telemetry telemetry) {
        telemetry.addData("Front Left Power", flMotor.getPower());
        telemetry.addData("Front Right Power", frMotor.getPower());

        if (constants.fourMotor) {
            telemetry.addData("Back left Power", blMotor.getPower());
            telemetry.addData("Back Right Power", brMotor.getPower());
        }
    }
    
    @NonNull
    @Override
    public String toString() {
        if (constants.fourMotor) {
            return String.format(Locale.ENGLISH,
                    "Tank(fourMotor=true, fl=%.1f, bl=%.1f, fr=%.1f, br=%.1f)", 
                    flMotor.getPower(), blMotor.getPower(), frMotor.getPower(), brMotor.getPower());
        } else {
            return String.format(Locale.ENGLISH,
                    "Tank(fourMotor=false, fl=%.1f, fr=%.1f)", 
                    flMotor.getPower(), frMotor.getPower());
        }
    }
}
