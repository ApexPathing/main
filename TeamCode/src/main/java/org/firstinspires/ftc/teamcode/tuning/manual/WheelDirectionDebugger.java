package org.firstinspires.ftc.teamcode.tuning.manual;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

import drivetrains.constants.MecanumConstants;

/**
 * Simple OpMode for testing the direction of each wheel on a mecanum drivetrain. Pressing A, B, X,
 * or Y will run the corresponding wheel at 30% power, allowing you to verify that the wheels are
 * set up correctly and spinning in the correct direction.
 *
 * @author Joel - 7842 Browncoats Alumni
 */
@TeleOp(name = "Wheel Direction Debugger", group = "Apex Pathing Tests")
public class WheelDirectionDebugger extends OpMode {
    DcMotor fl, fr, bl, br;

    @Override
    public void init() {
        MecanumConstants driveConstants = (MecanumConstants) new Constants().drivetrainConstants;
        fl = hardwareMap.get(DcMotor.class, driveConstants.flData.getName());
        fr = hardwareMap.get(DcMotor.class, driveConstants.frData.getName());
        bl = hardwareMap.get(DcMotor.class, driveConstants.blData.getName());
        br = hardwareMap.get(DcMotor.class, driveConstants.brData.getName());
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