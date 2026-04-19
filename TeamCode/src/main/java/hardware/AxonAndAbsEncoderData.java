package hardware;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Holds Data for the Servo and its corresponding Absolute Encoder
 * @author Xander Haemel - 31616 404 Not Found
 */
public class AxonAndAbsEncoderData {
    //servo stuff
    public String servoName;

    public CRServo.Direction direction;

    //encoderStuff
    public String absoluteEncoderName;

    public double minimumVoltage = 0;

    public double maximumVoltage = 5.0;


    public AxonAndAbsEncoderData(){

    }
    /**
     * sets the servo to a name
     * @param name is the String of the servo's name
     */
    public void setServoName(String name){
        this.servoName = name;
    }

    /**
     * sets the cr servo direction
     * @param direction is the new direction
     */
    public void setServoDirection(CRServo.Direction direction){
        this.direction = direction;
    }

    /**
     * sets the absolute encoder's name
     * @param name is a String
     */
    public void setAbsoluteEncoderName(String name){
        this.absoluteEncoderName = name;
    }

    /**
     * sets the minimum voltage for the abs encoder
     * @param voltage is the voltage in volts
     */
    public void setMinimumVoltage(double voltage){
        this.minimumVoltage = voltage;
    }

    /**
     * sets the maximum voltage for the abs encoder
     * @param voltage is the voltage in volts
     */
    public void setMaximumVoltage(double voltage){
        this.maximumVoltage = voltage;
    }



    //getters

    /**
     * gets the servo name
     * @return String ServoName
     */
    public String getServoName(){
        return servoName;
    }

    /**
     * gets the servo direction
     * @return FORWARD or REVERSE
     */
    public CRServo.Direction getServoDirection(){
        return direction;
    }

    /**
     * gets the absolute encoder name
     * @return String name
     */
    public String getAbsoluteEncoderName(){
        return absoluteEncoderName;
    }

    /**
     * gets the minimum voltage
     * @return the voltage in volts
     */
    public double getMinimumVoltage(){
        return minimumVoltage;
    }
    /**
     * gets the maximum voltage
     * @return the voltage in volts
     */
    public double getMaximumVoltage(){
        return maximumVoltage;
    }

    /*
     * gets kI term
     * @return the raw term
     */
}
