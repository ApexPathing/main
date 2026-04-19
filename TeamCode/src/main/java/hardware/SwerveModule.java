package hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import util.SwerveUnit;

/**
 * swerve module class
 * @author Xander Haemel - 31616  404 Not Found
 */
public class SwerveModule {
    public AxonAndAbsEncoderData servoConstants;

    public MotorEx motor;
    public ServoController servoController;

    /**
     * default constructor
     * @param hardwareMap the hardwareMapping
     * @param motor is the motor to give the class
     */
    public SwerveModule(HardwareMap hardwareMap, MotorEx motor){
        servoConstants = new AxonAndAbsEncoderData();
        servoController = new ServoController(servoConstants, hardwareMap);
        this.motor = motor;
    }

    /**
     * runs the motor to a power
     * @param power is the power to run to (0.0 to 1.0)
     */
    public void setPower(double power){
        motor.motor.setPower(power);
    }
    public void setPodAngleAndPower(SwerveUnit x){
        motor.motor.setPower(x.getMotorPower());
        servoController.setTargetPosition(x.getServoAngle());
    }
    /**
     * sets the pod to a new heading angle
     * @param angle is in degrees
     */
    public void setPodHeadingAngle(double angle) {
        servoController.setTargetPosition(angle);
    }

    /**
     * call this every loop
     */
    public void update(){
        servoController.update();
    }
    //getters

    /**
     * returns the swerve pod angle
     * @return the angle in degrees
     */
    public double getPodHeading(){
        return servoController.getCurrentPosition();
    }

    /**
     * gets the motors current power
     * @return a double from 0.0 - 1.0
     */
    public double getPower(){
        return motor.motor.getPower();
    }

    /**
     * returns current of the swerve motor
     * @return the current in amps
     */
    public double getMotorCurrent(){
        return motor.motor.getCurrent(CurrentUnit.AMPS);
    }




}
