package motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A class to seamlessly combine MotorMetaData with the hardware.
 * @author Atharv G - 13085 Bionic Dutch
 */
public class MotorEx {
    public DcMotorEx motor;
    public MotorMetaData data;

    public MotorEx(HardwareMap hardwareMap, MotorMetaData data) {
        motor = hardwareMap.get(DcMotorEx.class, data.getName());
        setRunMode(data.getRunMode());
        setDirection(data.getDirection());
        setBrakeMode(data.getBrakeMode());
    }

    public void setRunMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
        data.setRunMode(mode);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        data.setDirection(direction);
    }

    public void setBrakeMode(DcMotor.ZeroPowerBehavior mode) {
        motor.setZeroPowerBehavior(mode);
        data.setBrakeMode(mode);
    }
}
