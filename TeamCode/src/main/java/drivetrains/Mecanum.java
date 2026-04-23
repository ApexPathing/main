package drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

import drivetrains.constants.MecanumConstants;

/**
 * Mecanum drivetrain controller class
 *
 * @author Xander Haemel - 31616 - 404 Not Found
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Mecanum extends Drivetrain {
    MecanumConstants constants;

    // Motors
    DcMotorEx flMotor;
    DcMotorEx blMotor;
    DcMotorEx frMotor;
    DcMotorEx brMotor;

    /**
     * Creates a mecanum drivetrain
     * @param hardwareMap the hardware map to use for motor initialization
     * @param constants {@link MecanumConstants} object containing all tunable values and motor names/directions
     */
    public Mecanum(HardwareMap hardwareMap, @NonNull MecanumConstants constants){
        this.constants = constants;

        flMotor = this.constants.flData.build(hardwareMap);
        blMotor = this.constants.blData.build(hardwareMap);
        frMotor = this.constants.frData.build(hardwareMap);
        brMotor = this.constants.brData.build(hardwareMap);
    }

    protected boolean isRobotCentric() { return constants.robotCentric; }

    /**
     * Sets the power for each motor, normalizing the powers if any exceed the maximum allowed power.
     * @param flPower the power to set for the left front motor
     * @param blPower the power to set for the left rear motor
     * @param frPower the power to set for the right front motor
     * @param brPower the power to set for the right rear motor
     */
    private void setPowers(double flPower, double blPower, double frPower, double brPower) {
        // Normalize powers from -maxPower to maxPower if any exceed the max
        double max = Math.max(0, Math.abs(flPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(frPower));
        max = Math.max(max, Math.abs(brPower));
        if (max > constants.maxPower) {
            flPower = (flPower / max) * constants.maxPower;
            blPower = (blPower / max) * constants.maxPower;
            frPower = (frPower / max) * constants.maxPower;
            brPower = (brPower / max) * constants.maxPower;
        }

        // Normalize motor powers to not exceed the max current (if enabled)
        if (constants.maxCurrent < 0) {
            if (getTotalCurrent() > constants.maxCurrent) {
                double currentRatio = getTotalCurrent() / constants.maxCurrent;
                flPower /= currentRatio;
                frPower /= currentRatio;
                blPower /= currentRatio;
                brPower /= currentRatio;
            }
        }

        flMotor.setPower(flPower);
        blMotor.setPower(blPower);
        frMotor.setPower(frPower);
        brMotor.setPower(brPower);
    }

    public void moveWithVectors(double drive, double strafe, double turn) {
        double flPower = drive + strafe + turn;
        double blPower = drive - strafe + turn;
        double frPower = drive - strafe - turn;
        double brPower = drive + strafe - turn;

        setPowers(flPower, blPower, frPower, brPower);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    /**
     * @return the total motor current of the drivetrain in amps
     */
    private double getTotalCurrent(){
        return flMotor.getCurrent(CurrentUnit.AMPS) + frMotor.getCurrent(CurrentUnit.AMPS) +
                blMotor.getCurrent(CurrentUnit.AMPS)+ brMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Front Left Power", flMotor.getPower());
        telemetry.addData("Front Right Power", frMotor.getPower());
        telemetry.addData("Back left Power", blMotor.getPower());
        telemetry.addData("Back Right Power", brMotor.getPower());
    }
    
    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH,
                "Mecanum(fl=%.1f, bl=%.1f, fr=%.1f, br=%.1f)", 
                flMotor.getPower(), blMotor.getPower(), frMotor.getPower(), brMotor.getPower()
        );
    }
}
