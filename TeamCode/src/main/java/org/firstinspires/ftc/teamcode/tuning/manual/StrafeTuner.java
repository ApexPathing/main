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
 * OpMode for tuning the strafe controller with Panels. Hold X to move the robot 24 inches left,
 * hold B to move 6 inches right, and hold A to move it back to the start position. Adjust the
 * proportional gain, derivative gain, minimum power, and deadzone in Panels.
 *
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Configurable
@TeleOp(name = "Strafe Tuner", group = "Apex Pathing Tuning")
public class StrafeTuner extends OpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;
    private PDFLController controller;
    private PDFLController headingController;
    private JoinedTelemetry fullTelem;

    double target = 0;
    public static boolean maintainHeading; // Use the heading controller
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
        headingController = followerConstants.headingController;
        controller = followerConstants.strafeController;
        proportionalGain = controller.getCoefficients().kP;
        derivativeGain = controller.getCoefficients().kD;
        minPower = controller.getCoefficients().kL;
        deadzone = controller.getDeadzone();

        fullTelem.addLine(
                "Hold X to move left 24 inches, B to move right 6 inches, and A to move back to start position."
        );
        fullTelem.update();
    }

    private void moveToTarget(double target) {
        this.target = target;

        double turn = 0;
        if (maintainHeading) {
            double headingError = 0 - this.localizer.getPose().getHeading(); // Target heading is 0 degrees
            turn = headingController.calculate(headingError);
        } else {
            headingController.reset(); // Prevent derivative kick when not maintaining heading
        }

        double error = target - this.localizer.getPose().getY();
        this.drivetrain.moveWithVectors(0, this.controller.calculate(error), turn);
    }

    @Override
    public void loop() {
        localizer.update();

        controller.setCoefficients(new PDFLCoefficients(proportionalGain, derivativeGain, minPower));
        controller.setDeadzone(deadzone);

        if (gamepad1.x) { // Move 24 inches to the left when X is held
            moveToTarget(24);
        } else if (gamepad1.b) { // Move 6 inches to the right when B is held
            moveToTarget(-6);
        } else if (gamepad1.a) { // Move back to 0 when A is held
            moveToTarget(0);
        } else {
            // Prevent derivative kick
            controller.reset();
            headingController.reset();
            drivetrain.stop();
        }

        fullTelem.addData("Target: ", target);
        fullTelem.addData("Position: ", localizer.getPose().getY());
        fullTelem.update();
    }
}
