package Actuators;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import controllers.SquIDController;

/**
 * controls the servo with a built in feedback controller
 * @author Xander Haemel - 31616 404 Not Found
 */
public class ServoController {
    public AxonAndAbsEncoderData constants;
    CRServo servo;
    AnalogInput encoder;
    SquIDController controller;
    private double targetPosition;

    /**
     * default constructor
     * @param constants is the servo and abs encoder constants
     */
    public ServoController(AxonAndAbsEncoderData constants, HardwareMap hardwareMap){
        this.constants = constants;
        encoder = hardwareMap.get(AnalogInput.class, constants.absoluteEncoderName);
        servo = hardwareMap.get(CRServo.class, constants.getServoName());
        controller = new SquIDController(0,0,0);
        servo.setDirection(constants.direction);
    }

    /**
     * sets the servo to a new targetPosition
     * @param targetPosition is the angle new position
     */
    public void setTargetPosition(double targetPosition){
        this.targetPosition = targetPosition;
    }

    /**
     * gets the position of the axon servo
     * @return the position in degrees
     */
    public double getCurrentPosition(){
        return encoder.getVoltage()/3.3;
    }

    /**
     * updates pids and calculates power
     */
    public void update() {
        servo.setPower(controller.calculate(targetPosition - getCurrentPosition()));
    }




}
