package org.firstinspires.ftc.teamcode.tuning.manual;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import controllers.PDFLController;
import drivetrains.Drivetrain;
import localizers.Localizer;
import util.Pose;

/**
 * OpMode for tuning the heading controller with Panels. Hold A to turn the robot 180 degrees and
 * hold B to turn it back to the starting heading. Adjust the proportional gain, derivative gain,
 * minimum power, and deadzone in Panels.
 *
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
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
        Constants constants = new Constants();
        drivetrain = constants.buildOnlyDrivetrain(hardwareMap);
        localizer = constants.buildOnlyLocalizer(hardwareMap, Pose.zero());
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
