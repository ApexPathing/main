package motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorMetaData {
    private String name = "motor";
    private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    private DcMotor.ZeroPowerBehavior brakeMode = DcMotor.ZeroPowerBehavior.FLOAT;
    DcMotor.RunMode runMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;

    public MotorMetaData (String name, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior brakeMode, DcMotor.RunMode runMode) {
        this.name = name;
        this.direction = direction;
        this.brakeMode = brakeMode;
        this.runMode = runMode;
    }

    public MotorMetaData (String name, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior brakeMode) {
        this(name, direction, brakeMode, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public MotorMetaData(String name, DcMotorSimple.Direction direction) {
        this(name, direction, DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public MotorMetaData(String name) {
        this(name, DcMotorSimple.Direction.FORWARD);
    }
    public MotorMetaData() {
        this("motor");
    }

    public String getName() {
        return name;
    }
    public void setName(String name) {
        this.name = name;
    }

    public DcMotorSimple.Direction getDirection() {
        return direction;
    }
    public void setDirection(DcMotorSimple.Direction direction) {
        this.direction = direction;
    }

    public DcMotor.ZeroPowerBehavior getBrakeMode() {
        return brakeMode;
    }
    public void setBrakeMode(DcMotor.ZeroPowerBehavior mode) {
        this.brakeMode = mode;
    }

    public DcMotor.RunMode getRunMode() {
        return runMode;
    }
    public void setRunMode(DcMotor.RunMode mode) {
        this.runMode = mode;
    }
}