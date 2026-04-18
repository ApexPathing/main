package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class WheelDirectionDebugger extends OpMode {
    DcMotor fl, fr, bl, br;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, Constants.driveConstants.flData.getName());
        fr = hardwareMap.get(DcMotor.class, Constants.driveConstants.frData.getName());
        bl = hardwareMap.get(DcMotor.class, Constants.driveConstants.blData.getName());
        br = hardwareMap.get(DcMotor.class, Constants.driveConstants.brData.getName());
    }

    @Override
    public void loop() {
        runMotor(fl, gamepad1.a, "Front Left");
        runMotor(fr, gamepad1.b, "Front Right");
        runMotor(bl, gamepad1.x, "Back Left");
        runMotor(br, gamepad1.y, "Back Right");
    }

    private void runMotor(DcMotor motor, boolean btn, String name) {
        motor.setPower(btn ? 0.3 : 0);
        if (btn) telemetry.addLine(name + " Running!");
    }
}