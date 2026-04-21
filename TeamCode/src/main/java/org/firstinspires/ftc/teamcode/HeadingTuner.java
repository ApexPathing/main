package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import controllers.PDFLController;
import drivetrains.Drivetrain;
import drivetrains.Mecanum;
import localizers.Localizer;
import localizers.OTOS;
import util.Pose;

@Configurable
@TeleOp(name = "Heading Tuner", group = "Apex Pathing Tuning")
public class HeadingTuner extends OpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;
    double target = 0;
    public static double minPower;
    public static double deadzone;
    public static double proportionalGain;
    public static double derivativeGain;
    PDFLController controller;

    @Override
    public void init() {
        drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        localizer = new OTOS(hardwareMap, Constants.localizerConstants, new Pose(0,0,0));
        controller = new PDFLController(proportionalGain, derivativeGain, 0.0, minPower);
        controller.setDeadzone(deadzone);
        controller.useAsAngularController();
        telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        telemetry.addLine("Hold A to turn robot 180 and hold B to turn back to start heading.");
    }

    @Override
    public void loop() {
        localizer.update();

        if (gamepad1.a) {
            target = Math.PI;
            drivetrain.moveWithVectors(0,0, -controller.calculate(Math.PI - localizer.getPose().getHeading()));
        } else if (gamepad1.b) {
            target = 0;
            drivetrain.moveWithVectors(0,0, -controller.calculate(0 - localizer.getPose().getHeading()));
        } else {
            drivetrain.stop();
        }
        controller.setDeadzone(deadzone);
        controller.setPDFLCoefficients(proportionalGain, derivativeGain, 0.0, minPower);
        telemetry.addData("Target: ", target);
        telemetry.addData("Position: ", localizer.getPose().getHeading());
    }
}
