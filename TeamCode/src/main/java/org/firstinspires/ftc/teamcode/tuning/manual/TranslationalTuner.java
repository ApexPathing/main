package org.firstinspires.ftc.teamcode.tuning.manual;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import controllers.PDFLController;
import controllers.vector.PDLVectorController;
import drivetrains.Drivetrain;
import followers.constants.P2PFollowerConstants;
import localizers.Localizer;
import util.Pose;
import util.Vector;

/**
 * OpMode for tuning the translational controller with Panels. Hold A to drive the robot to (24, 24)
 * and hold B to drive the robot back to (0, 0). Adjust the proportional gain, derivative gain,
 * minimum power, and deadzone in Panels.
 *
 * @author Joel - 7842 Browncoats Alumni
 */
@Configurable
@TeleOp(name = "Translational Tuner", group = "Apex Pathing Tuning")
public class TranslationalTuner extends OpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;
    private Vector targetPos = new Vector();
    private double targetHeading;
    private PDFLController headingController;
    private PDLVectorController translationalController;
    public static double minPower;
    public static double deadzone;
    public static double proportionalGain;
    public static double derivativeGain;

    @Override
    public void init() {
        Constants constants = new Constants();
        drivetrain = constants.buildOnlyDrivetrain(hardwareMap);
        localizer = constants.buildOnlyLocalizer(hardwareMap, Pose.zero());
        P2PFollowerConstants followerConstants = (P2PFollowerConstants) constants.followerConstants;

        headingController = new PDFLController(
                followerConstants.headingGain,
                followerConstants.headingD,
                0.0, followerConstants.minPower
        );
        headingController.useAsAngularController();
        translationalController = new PDLVectorController(
                followerConstants.translationalGain,
                followerConstants.translationalD,
                followerConstants.minPower
        );
        telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        telemetry.addLine("Hold A to drive the robot to (24, 24) and hold B to drive the robot back to (0, 0).");
    }

    @Override
    public void loop() {
        localizer.update();
        Vector location = localizer.getPose().toVec();

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        if (a) {
            targetHeading = 0.0;
            targetPos = new Vector(24, 24);
        } else if (b) {
            targetHeading = 0.0;
            targetPos = new Vector();
        }

        translationalController.setPDLCoefficients(proportionalGain, derivativeGain, minPower);
        Vector error = targetPos.subtract(location);
        if (!a && !b) {
            drivetrain.stop();
            translationalController.reset();
        } else {
            Vector power = translationalController.calculate(error);
            drivetrain.moveWithVectors(power.getX(),power.getY(),-headingController.calculate(targetHeading - localizer.getPose().getHeading()));
        }

        telemetry.addData("Target: ", 0.0);
        telemetry.addData("Position: ", error.getMagnitude());
    }
}