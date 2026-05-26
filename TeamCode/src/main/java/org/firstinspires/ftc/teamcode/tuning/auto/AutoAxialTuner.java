package org.firstinspires.ftc.teamcode.tuning.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.concurrent.TimeUnit;

import controllers.PDFLController;
import drivetrains.Drivetrain;
import followers.constants.P2PFollowerConstants;
import localizers.Localizer;
import util.Pose;

// TODO: Reduce redundant boilerplate code by make base class for auto tuners
@TeleOp(name = "Auto Axial Tuner")
public class AutoAxialTuner extends LinearOpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;

    // Controllers
    private PDFLController axialController; // Axial controller
    private PDFLController headingController; // Heading lock controller

    private ElapsedTime timer;

    // Axial Tuning Variables
    public static double minPower;
    public static double proportionalGain;
    public static double derivativeGain;

    // The target distance to travel during Phase 1 and Phase 3 (48 inches = 2 FTC tiles)
    private final double TEST_DISTANCE = 48.0; // TODO: Make this adjustable by the user?

    @Override
    public void runOpMode() throws InterruptedException {
        // region init

        // Initialize hardware and controllers
        while (opModeInInit()) {
            Constants constants = new Constants();
            P2PFollowerConstants followerConstants = (P2PFollowerConstants) constants.setFollowerConstants();
            drivetrain = constants.buildOnlyDrivetrain(hardwareMap);
            localizer = constants.buildOnlyLocalizer(hardwareMap, Pose.zero());

            // Initialize Axial Controller
            axialController = new PDFLController(new PDFLController.PDFLCoefficients(proportionalGain, derivativeGain, 0.0, minPower));

            // Initialize Heading Controller to maintain a straight line
            headingController = new PDFLController(
                        new PDFLController.PDFLCoefficients(
                        followerConstants.headingCoeffs.kP,
                        followerConstants.headingCoeffs.kD,
                        0.0,
                        0.0
                    )
            );
            headingController.setAngularController();

            timer = new ElapsedTime();
            telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

            telemetry.addLine("WARNING: ENSURE ROBOT HAS AT LEAST 2 TILES OF CLEAR SPACE FORWARD AND BACKWARD");
            telemetry.update();
        }

        // endregion
        // region start

        timer.reset();
        localizer.setPose(new Pose(0, 0, 0));

        // endregion
        // region binary guess min-power tuner

        double maxGuess = 0.2;
        double minGuess = 0.0;
        final double initialGuess = (maxGuess + minGuess) / 2.0;

        double guess = initialGuess;
        double lastGuess = -1.0;
        double maxDetectedAxialVelocity = 9999;
        boolean hasMoved;
        final double HAS_MOVED_THRESHOLD = 0.1;

        // Phase 1: Binary search to find the minimum power required to overcome static friction (kS)
        while (opModeIsActive() && Math.abs(lastGuess - guess) > 0.01 && maxDetectedAxialVelocity > HAS_MOVED_THRESHOLD) {
            update();

            telemetry.addData("Phase", "1/3: Tuning Min Power (kS)");
            telemetry.addData("Current Power Guess", guess);
            telemetry.update();

            axialController.setCoefficients(new PDFLController.PDFLCoefficients(0, 0, 0, guess));

            // Calculate the distance to both ends of the test track
            double distToZero = Math.abs(localizer.getPose().getX() - 0);
            double distToTarget = Math.abs(localizer.getPose().getX() - TEST_DISTANCE);

            // Choose whichever target is further away to avoid deadzone issues
            double target = (distToTarget > distToZero) ? TEST_DISTANCE : 0;

            maxDetectedAxialVelocity = 0.0;

            timer.reset();
            while (opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 500) {
                update();
                maxDetectedAxialVelocity = Math.abs(Math.max(Math.abs(localizer.getVelocity().getX()), maxDetectedAxialVelocity));
                driveTo(target - localizer.getPose().getX());
            }

            hasMoved = maxDetectedAxialVelocity > HAS_MOVED_THRESHOLD;
            lastGuess = guess;

            // Adjust binary bounds based on whether the chassis moved
            if (hasMoved) {
                maxGuess = guess;
                guess = (guess + minGuess) / 2.0;
            } else {
                minGuess = guess;
                guess = (guess + maxGuess) / 2.0;
            }

            drivetrain.moveWithVectors(0,0,0);
            sleep(200);
        }

        final double kS = guess;

        // endregion
        // region Ziegler Nichol's method-based auto PID tuner

        timer.reset();

        double dt;
        double dv;
        double accel;
        double maxAccel = 0;
        double maxVel = 0;
        double lastVel = 0;
        double lastTime = System.nanoTime();
        double startTime = System.nanoTime();
        double timeStamp = 0;
        double velAtTimeStamp = 0;

        // Phase 2: Apply max power step input to calculate inflection point and system delays
        while (opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 2000) {
            update();

            telemetry.addData("Phase", "2/3: Z-N Step Response");
            telemetry.addData("Time Elapsed (s)", timer.time(TimeUnit.SECONDS));
            telemetry.addData("Max Accel Detected", maxAccel);
            telemetry.update();

            double curVel = localizer.getVelocity().getX();

            dt = (System.nanoTime() - lastTime) / 1.0e9;
            dv = (curVel - lastVel);

            if (dt > 1e-6) {
                accel = dv / dt;
            } else {
                accel = 0.0;
            }

            // Capture the exact moment of maximum acceleration (the inflection point)
            if (maxAccel < accel) {
                maxAccel = accel;
                timeStamp = (System.nanoTime() - startTime) / 1.0e9;
                velAtTimeStamp = curVel;
            }

            maxVel = Math.max(curVel, maxVel);
            lastVel = curVel;
            lastTime = System.nanoTime();

            // Full forward power, but applying heading correction to keep it straight
            double headingError = AngleUnit.normalizeRadians(0 - localizer.getPose().getHeading());
            drivetrain.moveWithVectors(1.0, 0, headingController.calculateFromError(headingError));
        }

        // Calculate Delay Time (L) based on the tangent line of the inflection point
        double L = timeStamp - (velAtTimeStamp / maxAccel);

        // Derive parallel PD gains using Ziegler-Nichols open-loop formulas
        double kP = 1.2 / (L * maxAccel);
        double kD = 0.6 / maxAccel;

        axialController.setCoefficients(new PDFLController.PDFLCoefficients(kP, kD, 0.0, kS));

        // endregion
        // region final verification

        // Calculate the distance to both ends of the test track
        double distToZeroVerify = Math.abs(localizer.getPose().getX() - 0);
        double distToTargetVerify = Math.abs(localizer.getPose().getX() - TEST_DISTANCE);

        // Choose whichever target is further away for the initial verification swing
        double verificationTarget = (distToTargetVerify > distToZeroVerify) ? TEST_DISTANCE : 0;

        timer.reset();

        // Phase 3: Toggle target continuously to observe system settling and overshoot
        while (opModeIsActive()) {
            update();

            // Toggle target every 3 seconds
            if (timer.time(TimeUnit.MILLISECONDS) > 3000) {
                verificationTarget = (verificationTarget == TEST_DISTANCE) ? 0 : TEST_DISTANCE;
                timer.reset();
            }

            double error = verificationTarget - localizer.getPose().getX();
            driveTo(error);

            telemetry.addData("Phase", "3/3: Final Verification");
            telemetry.addData("Status", "Tuning Complete. Observe behavior.");
            telemetry.addLine();
            telemetry.addData("Calculated kP", kP);
            telemetry.addData("Calculated kD", kD);
            telemetry.addData("Calculated kS", kS);
            telemetry.addLine();
            telemetry.addData("Target", verificationTarget);
            telemetry.addData("Current X", localizer.getPose().getX());
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // endregion
    }

    private void driveTo(double axialError) {
        double headingError = AngleUnit.normalizeRadians(0 - localizer.getPose().getHeading());
        double turnCorrection = headingController.calculateFromError(headingError);

        drivetrain.moveWithVectors(axialController.calculateFromError(axialError), 0, turnCorrection);
    }

    private void update() {
        localizer.update();
    }
}
