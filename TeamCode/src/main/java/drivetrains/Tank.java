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

    protected boolean isRobotCentric() {
        return this.constants.robotCentric;
    }

    public void moveWithVectors(double drive, double strafe, double turn) {
        // Tank isn't holonomic, ignore the strafe vector
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize powers if any exceed the max power
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > constants.maxPower) {
            leftPower = (leftPower / max) * constants.maxPower;
            rightPower = (rightPower / max) * constants.maxPower;
        }

        // Apply to motors
        setPowers(leftPower, rightPower);
    }

    /**
     * Sets the power for each side of the robot, normalizing the powers if any exceed the maximum
     * allowed power.
     * @param leftPower the power to set for the left motors
     * @param rightPower the power to set for the right motors
     */
    private void setPowers(double leftPower, double rightPower) {
        // Normalize powers from -maxPower to maxPower if any exceed the max
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > constants.maxPower) {
            leftPower = (leftPower / max) * constants.maxPower;
            rightPower = (rightPower / max) * constants.maxPower;
        }

        flMotor.setPower(leftPower);
        frMotor.setPower(rightPower);
        if (constants.fourMotor) {
            blMotor.setPower(leftPower);
            brMotor.setPower(rightPower);
        }
    }

    public void stop() { setPowers(0, 0); }

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
