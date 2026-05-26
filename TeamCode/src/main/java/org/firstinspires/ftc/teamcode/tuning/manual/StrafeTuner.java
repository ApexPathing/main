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
import util.Distance;
import util.Pose;

/**
 * OpMode for tuning the strafe controller with Panels. Hold X to move the robot 64 inches left,
 * hold B to move 6 inches right, and hold A to move it back to the start position. Adjust the
 * coefficients, deadzone, and tolerance in Panels. If maintainHeading is true, the robot will use
 * the heading controller to maintain its heading while strafing.
 *
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Sohum Arora - 22985 Paraducks
 */
@Configurable
@TeleOp(name = "Strafe Tuner", group = "Apex Pathing Tuning")
public class StrafeTuner extends OpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;
    private PDSController controller;
    private PDSController headingController;
    private JoinedTelemetry fullTelem;

    double target = 0;
    public static boolean maintainHeading; // Use the heading controller
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
        headingController = followerConstants.headingController;
        headingController.setTarget(0);
        controller = followerConstants.strafeController;
        kP = controller.getCoefficients().kP;
        kD = controller.getCoefficients().kD;
        kS = controller.getCoefficients().kS;
        kSDeadzone = controller.getCoefficients().kSDeadzone;
        outputDeadzone = controller.getDeadzone();
        tolerance = controller.getTolerance();

        fullTelem.addLine(
                "Hold X to move left 64 inches, B to move right 6 inches, and A to move back to the start position."
        );
        fullTelem.update();
    }

    private void moveToTarget(double target) {
        this.target = target;
        controller.setTarget(target);

        double turn = 0;
        if (maintainHeading) {
            turn = headingController.calculate(this.localizer.getPose().getHeading());
        } else {
            headingController.reset(); // Prevent derivative kick when not maintaining heading
        }

        this.rawOutput = controller.calculate(this.localizer.getPose().getY());
        this.drivetrain.moveWithVectors(0, this.rawOutput, turn);
    }

    @Override
    public void loop() {
        localizer.update();

        controller.setCoefficients(new PDSCoefficients(kP, kD, kS, kSDeadzone));
        controller.setDeadzone(outputDeadzone);
        controller.setTolerance(new Distance(tolerance)); // Inches

        if (gamepad1.x) { // Move 64 inches to the left when X is held
            moveToTarget(64);
        } else if (gamepad1.b) { // Move 6 inches to the right when B is held
            moveToTarget(-6);
        } else if (gamepad1.a) { // Move back to 0 when A is held
            moveToTarget(0);
        } else {
            // Prevent derivative kick
            controller.reset();
            headingController.reset();
            drivetrain.stop();
            wasAtTarget = false;
        }

        boolean atTarget = controller.isAtTarget();
        if (atTarget && !wasAtTarget) { // Gamepad rumble and Led green when at target
            gamepad1.rumble(0.5, 0.5, 100);
            gamepad1.setLedColor(0, 1, 0, 300);
        } else if (!atTarget) { // Led red when not at target
            gamepad1.setLedColor(1, 0, 0, 100);
        }
        wasAtTarget = atTarget;
        
        fullTelem.addData("Target: ", target);
        fullTelem.addData("Position: ", localizer.getPose().getY());
        fullTelem.addData("Error: ", controller.getError());
        fullTelem.addData("At Target: ", atTarget);
        fullTelem.addData("Raw Controller Output: ", rawOutput);
        fullTelem.addData("Drivetrain Output: ", drivetrain.toString());
        fullTelem.update();
    }
}
