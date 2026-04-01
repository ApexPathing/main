package motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorEx  {
    public DcMotorEx motor;
    public MotorMetaData data;

    public MotorEx(HardwareMap hardwareMap, MotorMetaData data) {
        motor = hardwareMap.get(DcMotorEx.class, data.getName());
        motor.setMode(data.getRunMode());
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
