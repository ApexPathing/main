package Drivetrains.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Tank Drive Constants Class
 * @author Xander Haemel - 31616- 404 Not Found
 */
public class TankConstants {
    //declerations
    //TODO select motor count
    public boolean FourMotorTankDrive = true;
    public boolean useBrakingMode = false; // Brake mode = true, Float mode = false
    public boolean useFeedForward = true; // Whether to use feedforward in the velocity controller TODO: USE THIS
    public boolean robotCentric = true; // Whether to use robot-centric controls (true) or field-centric controls (false) in TeleOp

    //motor names
    //TODO note for 2 motor tank just use the front motors
    public String lF_name = "leftFront";
    public String lR_name = "leftRear";
    public String rF_name = "rightFront";
    public String rR_name = "rightRear";

    //motor directions (back motors not needed for 2 motor drive)
    public DcMotorSimple.Direction lF_Direction = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction lR_Direction = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction rF_Direction = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction rR_Direction = DcMotorSimple.Direction.FORWARD;


    //maximumVelocity
    public final double ForwardVelocity = 60; //in per sec

    //heading velocity
    public final double HeadingVelocity = 6.82; // radians per second (1 rotation per second default)

    //max power
    public double MaxPower = 1.0; //1.0 default

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
        this.lF_name = leftFrontMotorName;
        return this;
    }
    /**
     * Sets the left back motor name
     * @param leftRearMotorName is the new name
     * @return this instance for chaining
     */
    public TankConstants setLeftRearMotorName(String leftRearMotorName){
        this.lR_name = leftRearMotorName;
        return this;
    }
    /**
     * Sets the right front motor name
     * @param rightFrontMotorName is the new name
     * @return this instance for chaining
     */
    public TankConstants setRightFrontMotorName(String rightFrontMotorName){
        this.rF_name = rightFrontMotorName;
        return this;
    }/**
     * Sets the left front motor name
     * @param rightRearMotorName is the new name
     * @return this instance for chaining
     */
    public TankConstants rightRearMotorName(String rightRearMotorName){
        this.rR_name = rightRearMotorName;
        return this;
    }


    //directions setters
    /**
     * Sets the left front motor direction
     * @param leftFrontMotorDirection is the new name
     * @return this instance for chaining
     */
    public TankConstants setLeftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection){
        this.lF_Direction = leftFrontMotorDirection;
        return this;
    }

    /**
     * Sets the left back motor name
     * @param leftRearMotorDirection is the new name
     * @return this instance for chaining
     */
    public TankConstants setLeftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection){
        this.lR_Direction = leftRearMotorDirection;
        return this;
    }
    /**
     * Sets the right front motor direction
     * @param rightFrontMotorDirection is the new name
     * @return this instance for chaining
     */
    public TankConstants setRightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection){
        this.rF_Direction = rightFrontMotorDirection;
        return this;
    }
    /**
     * Sets the right back motor direction
     * @param rightRearMotorDirection is the new name
     * @return this instance for chaining
     */
    public TankConstants setRightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection){
        this.rR_Direction = rightRearMotorDirection;
        return this;
    }
    /**
     * sets two motor or four motor tank drive
     */
    public void setFourMotorTankDrive(boolean enableFourMotorTankDrive){
        this.FourMotorTankDrive = enableFourMotorTankDrive;
    }








}
