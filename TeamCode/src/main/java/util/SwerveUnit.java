package util;

/**
 * Class for swerve
 * @author Xander Haemel - 31616 404 not found
 */
public class SwerveUnit {
    private double motorPower;
    private double servoAngle;
    /**
     * default constructor
     * @param motorPower is the motor power to save
     * @param servoAngle the servo angle to save
     */
    public SwerveUnit(double motorPower, double servoAngle){
        this.motorPower = motorPower;
        this.servoAngle = servoAngle;
    }

    /**
     * gets the stored motor power
     * @return is the power
     */
    public double getMotorPower(){
        return motorPower;
    }

    /**
     * gets the stored servo angle
     * @return servo angle
     */
    public double getServoAngle(){
        return servoAngle;
    }

    /**
     * sets the servo angle
     * @param angle is the angle in degrees
     */
    public void setServoAngle(double angle){
        this.servoAngle = angle;
    }

    /**
     * sets the motor to a power
     * @param motorPower is the motor power
     */
    public void setMotorSpeed(double motorPower){
        this.motorPower = motorPower;
    }

    /**
     * sets both constnats
     * @param motorPower is a double
     * @param servoAngle is an angle in degrees
     */
    public void setConstants(double motorPower, double servoAngle){
        this.motorPower = motorPower;
        this.servoAngle = servoAngle;
    }
}
