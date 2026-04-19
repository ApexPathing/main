package drivetrains.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import hardware.MotorEx;

/**
 * Represents one swerve pod: a drive motor + CRServo steering + analog encoder feedback.
 * @author Xander Haemel - 31616 404 not found
 * @author Sohum Arora - 22985 Paraducks
 */
public class SwerveModule {
    private final MotorEx driveMotor;
    private final CRServo steerServo;
    private final AnalogInput encoder; // 0–3.3V maps to 0–360 degrees

    private double targetAngle = 0;
    private double targetPower = 0;

    // P gain for steering loop — tune this on bot
    private static final double kP = 0.015;

    /**
     * @param hardwareMap  hardware map
     * @param driveMotor   pre-configured MotorEx for this pod
     * @param servoName    CRServo hardware name (e.g. "flServo")
     * @param encoderName  AnalogInput hardware name (e.g. "flEncoder")
     */
    public SwerveModule(HardwareMap hardwareMap, MotorEx driveMotor, String servoName, String encoderName) {
        this.driveMotor = driveMotor;
        this.steerServo = hardwareMap.get(CRServo.class, servoName);
        this.encoder = hardwareMap.get(AnalogInput.class, encoderName);
    }

    /**
     * Returns current pod heading in degrees [0, 360)
     */
    public double getPodHeading() {
        return (encoder.getVoltage() / 3.3) * 360.0;
    }

    /**
     * Store target angle and power — applied on update()
     */
    public void setPodAngleAndPower(SwerveUnit unit) {
        this.targetAngle = unit.getServoAngle();
        this.targetPower = unit.getMotorPower();
    }

    /**
     * Must be called every loop iteration.
     * Runs P control on steering and applies drive power.
     */
    public void update() {
        double current = getPodHeading();
        double error = targetAngle - current;
        // wrap to [-180, 180]
        error = error - (360.0 * Math.round(error / 360.0));

        double servoOutput = kP * error;
        servoOutput = Math.max(-1.0, Math.min(1.0, servoOutput));

        steerServo.setPower(servoOutput);
        driveMotor.setPower(targetPower);
    }
}