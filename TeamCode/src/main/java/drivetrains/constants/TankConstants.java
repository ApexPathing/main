package Drivetrains.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import motors.MotorMetaData;

/**
 * Tank Drive Constants Class
 * @author Xander Haemel - 31616- 404 Not Found
 */

@SuppressWarnings("unused")public class TankConstants {
    //declerations
    //TODO select motor count
    public boolean fourMotorTankDrive = true;
    public boolean useBrakingMode = false; // Brake mode = true, Float mode = false
    public boolean useFeedForward = true; // Whether to use feedforward in the velocity controller TODO: USE THIS
    public boolean robotCentric = true; // Whether to use robot-centric controls (true) or field-centric controls (false) in TeleOp

    //motor names
    //TODO note for 2 motor tank just use the front motors
    public MotorMetaData leftFrontData = new MotorMetaData("frontLeft");
    public MotorMetaData rightFrontData = new MotorMetaData("frontRight");
    public MotorMetaData leftRearData = new MotorMetaData("rearLeft");
    public MotorMetaData rightRearData = new MotorMetaData("rearRight");

    //maximumVelocity
    public final double forwardVelocity = 60; //in per sec

    //heading velocity
    public final double headingVelocity = 6.82; // radians per second (1 rotation per second default)

    //max power
    public double maxPower = 1.0; //1.0 default

    //setters

    /**
     * defualt constructor
     */
    public TankConstants(){}
    /**
     * Sets the left front motor name
     * @param leftFrontMotorName is the new name
     * @return this instance for chaining
     */
    public TankConstants setLeftFrontMotorName(String leftFrontMotorName){
        leftFrontData.setName(leftFrontMotorName);
        return this;
    }
    /**
     * Sets the left back motor name
     * @param leftRearMotorName is the new name
     * @return this instance for chaining
     */
    public TankConstants setLeftRearMotorName(String leftRearMotorName){
        this.leftRearData.setName(leftRearMotorName);
        return this;
    }
    /**
     * Sets the right front motor name
     * @param rightFrontMotorName is the new name
     * @return this instance for chaining
     */
    public TankConstants setRightFrontMotorName(String rightFrontMotorName){
        this.rightFrontData.setName(rightFrontMotorName);
        return this;
    }/**
     * Sets the left front motor name
     * @param rightRearMotorName is the new name
     * @return this instance for chaining
     */
    public TankConstants setRightRearMotorName(String rightRearMotorName){
        this.rightRearData.setName(rightRearMotorName);
        return this;
    }


    //directions setters
    /**
     * Sets the left front motor direction
     * @param leftFrontMotorDirection is the new name
     * @return this instance for chaining
     */
    public TankConstants setLeftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection){
        this.leftFrontData.setDirection(leftFrontMotorDirection);
        return this;
    }

    /**
     * Sets the left back motor name
     * @param leftRearMotorDirection is the new name
     * @return this instance for chaining
     */
    public TankConstants setLeftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection){
        this.leftRearData.setDirection(leftRearMotorDirection);
        return this;
    }
    /**
     * Sets the right front motor direction
     * @param rightFrontMotorDirection is the new name
     * @return this instance for chaining
     */
    public TankConstants setRightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection){
        this.rightFrontData.setDirection(rightFrontMotorDirection);
        return this;
    }
    /**
     * Sets the right back motor direction
     * @param rightRearMotorDirection is the new name
     * @return this instance for chaining
     */
    public TankConstants setRightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection){
        this.rightRearData.setDirection(rightRearMotorDirection);
        return this;
    }
    /**
     * sets two motor or four motor tank drive
     */
    public void setFourMotorTankDrive(boolean enableFourMotorTankDrive) {
        this.fourMotorTankDrive = enableFourMotorTankDrive;
    }
}
