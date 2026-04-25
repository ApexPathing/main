package org.firstinspires.ftc.teamcode.tuning.manual;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import controllers.PDFLController.PDFLCoefficients;
import controllers.PDFLController;
import drivetrains.Drivetrain;
import followers.constants.P2PFollowerConstants;
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
    private PDFLController controller;
    private JoinedTelemetry fullTelem;

    double target = 0;
    public static double deadzone;
    public static double proportionalGain; // kP
    public static double derivativeGain; // kD
    public static double minPower; // kL

    @Override
    public void init() {
        // Build constants, drivetrain, localizer, and telemetry
        Constants constants = new Constants();
        drivetrain = constants.buildOnlyDrivetrain(hardwareMap);
        localizer = constants.buildOnlyLocalizer(hardwareMap, Pose.zero());
        fullTelem = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        // These controllers use the coefficients from the constants class
        P2PFollowerConstants followerConstants = (P2PFollowerConstants) constants.setFollowerConstants();

        // Extract the controllers, coefficients, and deadzone from the constants class
        // Note .useAngularController() is called by constants
        controller = followerConstants.headingController;
        proportionalGain = controller.getCoefficients().kP;
        derivativeGain = controller.getCoefficients().kD;
        minPower = controller.getCoefficients().kL;
        deadzone = controller.getDeadzone();

        fullTelem.addLine(
                "Hold X to rotate 180 degrees, B to rotate to -45 degrees. and A to move back to start position."
        );
        fullTelem.update();
    }

    private void moveToTarget(double target) {
        this.target = target;
        double error = target - this.localizer.getPose().getHeading();
        this.drivetrain.moveWithVectors(0, 0, -this.controller.calculate(error));
    }

    @Override
    public void loop() {
        localizer.update();

        controller.setCoefficients(new PDFLCoefficients(proportionalGain, derivativeGain, minPower));
        controller.setDeadzone(deadzone);

        if (gamepad1.x) { // Move to 180 degrees when X is held
            moveToTarget(Math.PI);
        } else if (gamepad1.b) { // Move to -45 (315) degrees when B is held
            moveToTarget(-Math.PI / 4);
        } else if (gamepad1.a) { // Move back to 0 degrees when A is held
            moveToTarget(0);
        } else {
            controller.reset();
            drivetrain.stop();
        }

        fullTelem.addData("Target: ", target);
        fullTelem.addData("Position: ", localizer.getPose().getHeading());
        fullTelem.update();
    }
}
