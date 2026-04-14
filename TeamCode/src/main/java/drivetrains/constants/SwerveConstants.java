package drivetrains.constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import motors.MotorMetaData;

/**
 * @author Xander Haemel 31616
 * swerve constants file
 */
public class SwerveConstants {
    //motors
    public MotorMetaData flData = new MotorMetaData("front_left_drive");
    public MotorMetaData blData = new MotorMetaData("back_left_drive");
    public MotorMetaData frData = new MotorMetaData("front_right_drive");
    public MotorMetaData brData = new MotorMetaData("back_right_drive");
    //servos
    public String flServoName = "flServo";
    public String blServoName = "blServo";
    public String frServoName = "frServo";
    public String brServoName = "brServo";
    //encoders
    public String flEncoderName = "flEncoder";
    public String blEncoderName = "blEncoder";
    public String frEncoderName = "frEncoder";
    public String brEncoderName = "brEncoder";

    //variable constants
    public double ServoMaxPower = 1;//range 0-1
    public double MotorMaxPower = 1; //range 0-1
    public double MaxSpeed = 60; //inches per second
    public double MotorCurrentLimit = 1000; //milliamps, per motor
    public double ServoCurrentLimit = 1000; //milliamps per servo

    /**
     * default constructor
     */
    public SwerveConstants() {

    }

    /**
     * sets the left front servo name
     * @param name is String name
     * @return this instance for stacking
     */
    public SwerveConstants setFrontLeftServoName(String name){
        flServoName = name;
        return this;
    }
    /**
     * sets the left back servo name
     * @param name is String name
     * @return this instance for stacking
     */
    public SwerveConstants setBackLeftServoName(String name){
        blServoName = name;
        return this;
    }
    /**
     * sets the right front servo name
     * @param name is String name
     * @return this instance for stacking
     */
    public SwerveConstants setFrontRightServoName(String name){
        frServoName = name;
        return this;
    }
    /**
     * sets the back right servo name
     * @param name is String name
     * @return this instance for stacking
     */
    public SwerveConstants setBackRightServoName(String name){
        brServoName = name;
        return this;
    }

    //motors------
    /**
     * Sets the left front motor name. Default: "front_left_drive"
     * @param name the name of the left front motor
     * @return this instance for chaining
     */
    public SwerveConstants setFrontLeftMotorName(String name) {
        this.flData.setName(name);
        return this;
    }
    /**
     * Sets the left rear motor name. Default: "back_left_drive"
     * @param name the name of the left rear motor
     * @return this instance for chaining
     */
    public SwerveConstants setBackLeftMotorName(String name) {
        this.blData.setName(name);
        return this;
    }
    /**
     * Sets the right front motor name. Default: "front_right_drive"
     * @param name the name of the right front motor
     * @return this instance for chaining
     */
    public SwerveConstants setFrontRightMotorName(String name) {
        this.frData.setName(name);
        return this;
    }
    /**
     * Sets the right rear motor name. Default: "back_right_drive"
     * @param name the name of the right rear motor
     * @return this instance for chaining
     */
    public SwerveConstants setBackRightMotorName(String name) {
        this.brData.setName(name);
        return this;
    }
    //directions
    /**
     * Default direction is FORWARD.
     * @param reversed whether the front left motor is reversed
     * @return this instance for chaining
     */
    public SwerveConstants setFrontLeftReversed(boolean reversed) {
        this.flData.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }
    /**
     * Default direction is FORWARD.
     * @param reversed whether the back left motor is reversed
     * @return this instance for chaining
     */
    public SwerveConstants setBackLeftReversed(boolean reversed) {
        this.blData.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }
    /**
     * Default direction is FORWARD.
     * @param reversed whether the front right motor is reversed
     * @return this instance for chaining
     */
    public SwerveConstants setFrontRightReversed(boolean reversed) {
        this.frData.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }
    /**
     * Default direction is FORWARD.
     * @param reversed whether the back right motor is reversed
     * @return this instance for chaining
     */
    public SwerveConstants setBackRightReversed(boolean reversed) {
        this.brData.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }
    //other

    /**
     * sets the max power of the motors on your swerve drivetrain
     * @param power is a double from 0.0 - 1.0
     * @return this instance for chaining
     */
    public SwerveConstants setMotorMaxPower(double power){
        this.MotorMaxPower = power;
        return this;
    }
    /**
     * sets the max power of the servos on your swerve drivetrain
     * @param power is a double from 0.0 - 1.0
     * @return this instance for chaining
     */
    public SwerveConstants setServoMaxPower(double power){
        this.ServoMaxPower = power;
        return this;
    }

    /**
     * sets the motor power limit, for each motor
     * @param milliamps is the limit in milliamps
     * @return this instance for chaining
     */
    public SwerveConstants setMotorCurrentLimit(int milliamps){
        this.MotorCurrentLimit = milliamps;
        return this;
    }
    /**
     * sets the servo power limit, for each one
     * @param milliamps is the limit in milliamps
     * @return this instance for chaining
     */
    public SwerveConstants setServoCurrentLimit(int milliamps){
        this.ServoCurrentLimit = milliamps;
        return this;
    }






}
