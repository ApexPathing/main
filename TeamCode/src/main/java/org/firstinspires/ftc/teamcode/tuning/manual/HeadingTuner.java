package org.firstinspires.ftc.teamcode.tuning.manual;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import controllers.PDSController.PDSCoefficients;
import controllers.PDSController;
import drivetrains.Drivetrain;
import followers.constants.P2PFollowerConstants;
import localizers.Localizer;
import util.Angle;
import util.Pose;

/**
 * OpMode for tuning the heading controller with Panels. Hold A to turn the robot 180 degrees and
 * hold B to turn it back to the starting heading. Adjust the proportional gain, derivative gain,
 * minimum power, and deadzone in Panels.
 *
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Sohum Arora - 22985 Paraducks
 */
@Configurable
@TeleOp(name = "Heading Tuner", group = "Apex Pathing Tuning")
public class HeadingTuner extends OpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;
    private PDSController controller;
    private JoinedTelemetry fullTelem;

    double target = 0;
    public static double kP;
    public static double kD;
    public static double kS;
    public static double kSDeadzone;
    public static double outputDeadzone;
    public static double tolerance; // Tolerance for being at the target (inches)

    private boolean wasAtTarget = false;
    private double rawOutput;

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
        kP = controller.getCoefficients().kP;
        kD = controller.getCoefficients().kD;
        kS = controller.getCoefficients().kS;
        kSDeadzone = controller.getCoefficients().kSDeadzone;
        outputDeadzone = controller.getDeadzone();
        tolerance = controller.getTolerance();

        fullTelem.addLine(
                "Hold X to rotate 180 degrees, B to rotate to -45 degrees. and A to move back to the start position."
        );
        fullTelem.update();
    }

    private void moveToTarget(double target) {
        this.target = target;
        controller.setTarget(target);
        this.rawOutput = this.controller.calculate(this.localizer.getPose().getHeading());
        this.drivetrain.moveWithVectors(0, 0, rawOutput);
    }

    @Override
    public void loop() {
        localizer.update();

        controller.setCoefficients(new PDSCoefficients(kP, kD, kS, kSDeadzone));
        controller.setDeadzone(outputDeadzone);
        controller.setTolerance(new Angle(tolerance));

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

        boolean atTarget = controller.isAtTarget();
        if (atTarget && !wasAtTarget) { //Gamepad rumble and Led green when at target
            gamepad1.rumble(0.5, 0.5, 100);
            gamepad1.setLedColor(0, 1, 0, 300);
        } else if (!atTarget) { //Led red when not at target
            gamepad1.setLedColor(1, 0, 0, 100);
        }
        wasAtTarget = atTarget;

        fullTelem.addData("Target: ", target);
        fullTelem.addData("Position: ", localizer.getPose().getHeading());
        fullTelem.addData("Error: ", controller.getError());
        fullTelem.addData("At Target: ", atTarget);
        fullTelem.addData("Raw Controller Output: ", rawOutput);
        fullTelem.addData("Drivetrain Output: ", drivetrain.toString());
        fullTelem.update();
    }
}
