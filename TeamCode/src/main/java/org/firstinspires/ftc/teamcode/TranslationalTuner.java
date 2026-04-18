package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import controllers.PDFLController;
import controllers.VectorControllers.PDLVectorController;
import drivetrains.Drivetrain;
import drivetrains.Mecanum;
import localizers.Localizer;
import localizers.OTOS;
import util.Pose;
import util.Vector;

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
        drivetrain = new Mecanum(hardwareMap, Constants.driveConstants);
        localizer = new OTOS(hardwareMap, Constants.localizerConstants, new Pose(0,0,0));
        headingController = new PDFLController(
                Constants.followerConstants.headingGain,
                Constants.followerConstants.headingD,
                0.0, Constants.followerConstants.minPower
        );
        headingController.useAsAngularController();
        translationalController = new PDLVectorController(
                Constants.followerConstants.translationalGain,
                Constants.followerConstants.translationalD,
                Constants.followerConstants.minPower
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