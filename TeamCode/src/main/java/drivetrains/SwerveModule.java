package drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

import drivetrains.constants.SwerveModuleConstants;

/**
 * A swerve module consists of a drive motor, a steering CRServo, and an analog encoder.
 * This should NOT be used directly as a drivetrain. It is meant to be used by the {@link Swerve}
 * drivetrain class, which will handle the kinematics and control of multiple swerve modules.
 *
 * @author Xander Haemel - 31616 404 not found
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class SwerveModule {
    private final SwerveModuleConstants constants;
    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final AnalogInput encoder; // 0–3.3V maps to 0–360 degrees

    private double targetAngle = 0;
    private double targetPower = 0;
    private double lastTargetPower = 0;
    private double lastSteerError = 0;

    private static final double voltageToDegrees = 360.0 / 3.3; // Degrees = voltage * this

    /**
     * @param hardwareMap the hardware map to use for initializing the module
     * @param constants the {@link SwerveModuleConstants} containing the configuration
     */
    public SwerveModule(HardwareMap hardwareMap, SwerveModuleConstants constants) {
        this.constants = constants;
        this.driveMotor = this.constants.motorData.build(hardwareMap);
        this.steerServo = hardwareMap.get(CRServo.class, this.constants.servoName);
        this.encoder = hardwareMap.get(AnalogInput.class, this.constants.encoderName);
    }

    /** @param behavior the zero power behavior of the drive motor. */
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        driveMotor.setZeroPowerBehavior(behavior);
    }

    /**
     * @return current drive motor power, from -1 to 1
     */
    public double getDrivePower() { return targetPower; }

    /**
     * @return current pod heading in degrees [0, 360)
     */
    public double getAngle() { return encoder.getVoltage() * voltageToDegrees; }

    /**
     * @param power power to apply to the drive motor, from -1 to 1
     */
    private void setDrivePower(double power) { this.targetPower = power; }

    /**
     * @param angle desired pod heading in degrees [0, 360)
     */
    private void setUnoptimizedTargetAngle(double angle) { this.targetAngle = angle; }

    /**
     * Sets the target angle and power for this module, optimizing the angle to minimize rotation.
     * @param targetAngle desired pod heading in degrees [0, 360] (will be optimized to minimize rotation)
     * @param targetPower power to apply to the drive motor, from -1 to 1
     */
    public void setTargets(double targetAngle, double targetPower) {
        double delta = targetAngle - getAngle();
        double wrappedDelta = delta - (360 * Math.round(delta / 360.0));
        if (Math.abs(wrappedDelta) > 90) {
            targetPower *= -1;
            wrappedDelta -= Math.copySign(180, wrappedDelta);
        }

        this.setUnoptimizedTargetAngle(getAngle() + wrappedDelta);
        this.setDrivePower(targetPower);
    }

    /**
     * sets the target angle of the module, while optimizing the angle
     * @param targetAngle is the desired pod heading in degrees [0, 360)
     */
    public void setOptimizedTargetAngle(double targetAngle){
        double delta = targetAngle - getAngle();
        double wrappedDelta = delta - (360 * Math.round(delta / 360.0));
        if (Math.abs(wrappedDelta) > 90) {
            targetPower *= -1;
            wrappedDelta -= Math.copySign(180, wrappedDelta);
        }

        this.setUnoptimizedTargetAngle(getAngle() + wrappedDelta);
    }

    /**
     * Sets the drive power only
     * @param targetPower is the desired targetPower of the wheel
     */
    public void setOptimizedTargetPower(double targetPower){
        this.setDrivePower(targetPower);
    }

    /**
     * Sets the target angle and power for this module without optimizing the angle.
     * This will cause the module to always rotate in the direction of the target angle,
     * even if it means rotating more than 90 degrees.
     * @param targetAngle desired pod heading in degrees [0, 360)
     * @param targetPower power to apply to the drive motor, from -1 to 1
     */
    public void setUnoptimizedTargets(double targetAngle, double targetPower) {
        this.setUnoptimizedTargetAngle(targetAngle);
        this.setDrivePower(targetPower);
    }

    /**
     * Updates the steering servo and drive motor power only if needed.
     */
    public void update() {
        double error = targetAngle - getAngle();
        //don't set anything new unless the angle or power has changed
        if (error != lastSteerError) {
            lastSteerError = error; // Save unwrapped error
            error -= (360.0 * Math.round(error / 360.0)); // Wrap to [-180, 180]
            steerServo.setPower(Math.max(-1.0, Math.min(1.0,  this.constants.steeringPGain * error)));
        }
        if (targetPower != lastTargetPower) {
            driveMotor.setPower(targetPower);
            lastTargetPower = targetPower;
        }
    }

    /**
     * Stop pod movement by setting target power to 0 and updating to apply.
     * This will NOT change the target angle, so the pod will hold its position.
     */
    public void stop() { this.setDrivePower(0); this.update(); }



    /**
     * use in telemetry and other places where we need a string with the swerve module results or data
     */
    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "SwerveModule(angle=%.1f, power=%.1f)",
                getAngle(), targetPower);
    }

    /**
     * gets the amount the current is using
     * @return this in Amps
     */
    public double getCurrent(){
        return driveMotor.getCurrent(CurrentUnit.AMPS);
    }
}