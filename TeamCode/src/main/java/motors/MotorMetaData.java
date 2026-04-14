package motors;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * A class to hold metadata about a motor, such as its name, direction, brake mode, and run mode.
 * @author Atharv G - 13085 Bionic Dutch
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class MotorMetaData {
    private String name;
    private DcMotorSimple.Direction direction;
    private ZeroPowerBehavior brakeMode;
    private RunMode runMode;

    /**
     * Create a MotorMetaData with the given parameters.
     * @param name The name of the motor in the hardware map.
     * @param direction The direction of the motor (FORWARD or REVERSE).
     * @param brakeMode The zero power behavior of the motor (BRAKE or FLOAT).
     * @param runMode The run mode of the motor (RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, etc.).
     */
    public MotorMetaData(String name, DcMotorSimple.Direction direction,
                         ZeroPowerBehavior brakeMode, RunMode runMode) {
        this.name = name;
        this.direction = direction;
        this.brakeMode = brakeMode;
        this.runMode = runMode;
    }

    /**
     * Create a default MotorMetaData with the given name.
     * It has a default direction (FORWARD), brake mode (BRAKE), and run mode (RUN_WITHOUT_ENCODER).
     * @param name The name of the motor in the hardware map.
     */
    public MotorMetaData(String name) {
        this(name, DcMotorSimple.Direction.FORWARD,
                ZeroPowerBehavior.BRAKE, RunMode.RUN_WITHOUT_ENCODER
        );
    }

    /**
     * Create a default MotorMetaData configuration with the name "motor".
     * It has a default direction (FORWARD), brake mode (BRAKE), and run mode (RUN_WITHOUT_ENCODER).
     */
    public MotorMetaData() {
        this("motor", DcMotorSimple.Direction.FORWARD,
                ZeroPowerBehavior.BRAKE, RunMode.RUN_WITHOUT_ENCODER
        );
    }

    /**
     * @return The name of the motor in the hardware map.
     */
    public String getName() {
        return name;
    }

    /**
     * @param name The name of the motor in the hardware map.
     */
    public void setName(String name) {
        this.name = name;
    }

    /**
     * @return The direction of the motor (FORWARD or REVERSE).
     */
    public DcMotorSimple.Direction getDirection() {
        return direction;
    }

    /**
     * @param direction The direction of the motor (FORWARD or REVERSE).
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        this.direction = direction;
    }

    /**
     * @return The zero power behavior of the motor (BRAKE or FLOAT).
     */
    public ZeroPowerBehavior getBrakeMode() {
        return brakeMode;
    }

    /**
     * @param mode The zero power behavior of the motor (BRAKE or FLOAT).
     */
    public void setBrakeMode(ZeroPowerBehavior mode) {
        this.brakeMode = mode;
    }

    /**
     * @return The run mode of the motor (RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, etc.).
     */
    public RunMode getRunMode() {
        return runMode;
    }

    /**
     * @param mode The run mode of the motor (RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, etc.).
     */
    public void setRunMode(RunMode mode) {
        this.runMode = mode;
    }
}