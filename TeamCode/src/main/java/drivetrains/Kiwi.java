package drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

import drivetrains.constants.KiwiConstants;

/**
 * Kiwi drivetrain controller class
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Kiwi extends Drivetrain {
    KiwiConstants constants;

    // Motors
    DcMotorEx frMotor;
    DcMotorEx bMotor;
    DcMotorEx flMotor;

    private final double sqrt3over2 = Math.sqrt(3) / 2;

    /**
     * Creates a kiwi drivetrain
     * @param hardwareMap the hardware map to use for motor initialization
     * @param constants {@link KiwiConstants} object containing all tunable values and motor names/directions
     */
    public Kiwi(HardwareMap hardwareMap, @NonNull KiwiConstants constants){
        this.constants = constants;

        frMotor = this.constants.frMotorData.build(hardwareMap);
        bMotor = this.constants.bMotorData.build(hardwareMap);
        flMotor = this.constants.flMotorData.build(hardwareMap);
    }

    protected boolean isRobotCentric() { return constants.robotCentric; }

    /**
     * Sets the power for each motor, normalizing the powers if any exceed the maximum allowed power.
     * @param frMotorPower the desired power for motor 1
     * @param bMotorPower the desired power for motor 2
     * @param flMotorPower the desired power for motor 3
     */
    private void setPowers(double frMotorPower, double bMotorPower, double flMotorPower) {
        // Normalize powers from -maxPower to maxPower if any exceed the max
        double max = Math.max(0, Math.abs(frMotorPower));
        max = Math.max(max, Math.abs(bMotorPower));
        max = Math.max(max, Math.abs(flMotorPower));
        if (max > constants.maxPower) {
            frMotorPower = (frMotorPower / max) * constants.maxPower;
            bMotorPower = (bMotorPower / max) * constants.maxPower;
            flMotorPower = (flMotorPower / max) * constants.maxPower;
        }

        // Normalize motor powers to not exceed the max current (if enabled)
        if (constants.maxCurrent > 0) {
            if (getTotalCurrent() > constants.maxCurrent) {
                double currentRatio = getTotalCurrent() / constants.maxCurrent;
                frMotorPower /= currentRatio;
                bMotorPower /= currentRatio;
                flMotorPower /= currentRatio;
            }
        }

        frMotor.setPower(frMotorPower);
        bMotor.setPower(bMotorPower);
        flMotor.setPower(flMotorPower);
    }

    public void moveWithVectors(double drive, double strafe, double turn) {
        // Video explaining Kiwi kinematics: https://www.youtube.com/watch?v=n6TWzzj74gk&t=27
        double frMotorPower = -(strafe / 2) + (drive * sqrt3over2) + turn;
        double bMotorPower = strafe + turn;
        double flMotorPower = -(strafe / 2) - (drive * sqrt3over2) + turn;

        setPowers(frMotorPower, bMotorPower, flMotorPower);
    }

    public void stop() {
        setPowers(0, 0, 0);
    }

    /**
     * @return the total motor current of the drivetrain in amps
     */
    private double getTotalCurrent(){
        return frMotor.getCurrent(CurrentUnit.AMPS) + bMotor.getCurrent(CurrentUnit.AMPS) +
                flMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Motor 1 Power", frMotor.getPower());
        telemetry.addData("Motor 2 Power", bMotor.getPower());
        telemetry.addData("Motor 3 Power", flMotor.getPower());
    }
    
    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH,
                "Kiwi(fr=%.2f, b=%.2f, fl=%.2f)",
                frMotor.getPower(), bMotor.getPower(), flMotor.getPower()
        );
    }
}
