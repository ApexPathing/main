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
import localizers.Localizer;
import util.Pose;

@TeleOp(name = "Auto Heading Tuner")
public class AutoHeadingTuner extends LinearOpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;
    private PDFLController controller;
    private ElapsedTime timer;
    public static double minPower;
    public static double deadzone;
    public static double proportionalGain;
    public static double derivativeGain;

    @Override
    public void runOpMode() throws InterruptedException {
        // region init

        // Initialize hardware and controllers
        while (opModeInInit()) {
            Constants constants = new Constants();
            drivetrain = constants.buildOnlyDrivetrain(hardwareMap);
            localizer = constants.buildOnlyLocalizer(hardwareMap, Pose.zero());
            controller = new PDFLController(proportionalGain, derivativeGain, 0.0, minPower);
            controller.setDeadzone(deadzone);
            controller.useAsAngularController();
            timer = new ElapsedTime();
            telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

            telemetry.addLine("WARNING ENSURE ROBOT IS IN CLEAR AREA BEFORE BEGINNING TUNING");
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
        double maxDetectedAngularVelocity = 9999;
        boolean hasMoved;
        final double HAS_MOVED_THRESHOLD = 0.01; // TODO: Verify that this threshold is good

        // Phase 1: Binary search to find the minimum power required to overcome static friction (kS)
        while (opModeIsActive() && Math.abs(lastGuess - guess) > 0.05 && maxDetectedAngularVelocity > HAS_MOVED_THRESHOLD) {
            update();

            telemetry.addData("Phase", "1/3: Tuning Min Power (kS)");
            telemetry.addData("Current Power Guess", guess);
            telemetry.update();

            controller.setPDFLCoefficients(0, 0, 0, guess);

            // Calculate the shortest-path distance to both targets
            double distToZero = Math.abs(AngleUnit.normalizeRadians(localizer.getPose().getHeading() - 0));
            double distToPi = Math.abs(AngleUnit.normalizeRadians(localizer.getPose().getHeading() - Math.PI));

            // Choose whichever target is further away to avoid deadzone issues
            double target = (distToPi > distToZero) ? Math.PI : 0;

            maxDetectedAngularVelocity = 0.0;

            timer.reset();
            while (opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 500) { // TODO: Try shrinking this threshold for faster tuning
                update();
                maxDetectedAngularVelocity = Math.abs(Math.max(Math.abs(localizer.getVelocity().getHeading()), maxDetectedAngularVelocity));
                turnTo(target - localizer.getPose().getHeading());
            }

            hasMoved = maxDetectedAngularVelocity > HAS_MOVED_THRESHOLD;
            lastGuess = guess;

            // Adjust binary bounds based on whether the chassis moved
            if (hasMoved) {
                maxGuess = guess;
                guess = (guess + minGuess) / 2.0;
            } else {
                minGuess = guess;
                guess = (guess + maxGuess) / 2.0;
            }
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
        while (opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 2500) {
            update();

            telemetry.addData("Phase", "2/3: Z-N Step Response");
            telemetry.addData("Time Elapsed (s)", timer.time(TimeUnit.SECONDS));
            telemetry.addData("Max Accel Detected", maxAccel);
            telemetry.update();

            double curVel = localizer.getVelocity().getHeading();

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

            drivetrain.moveWithVectors(0, 0, 1.0); // Full rotational power
        }

        // Calculate Delay Time (L) based on the tangent line of the inflection point
        double L = timeStamp - (velAtTimeStamp / maxAccel);

        // Derive parallel PD gains using Ziegler-Nichols open-loop formulas
        double kP = 1.2 / (L * maxAccel);
        double kD = 0.6 / maxAccel;

        controller.setPDFLCoefficients(kP, kD, 0.0, kS);

        // endregion
        // region final verification

        // Calculate the shortest-path distance to both targets
        double distToZero = Math.abs(AngleUnit.normalizeRadians(localizer.getPose().getHeading() - 0));
        double distToPi = Math.abs(AngleUnit.normalizeRadians(localizer.getPose().getHeading() - Math.PI));

        // Choose whichever target is further away for the initial verification swing
        double verificationTarget = (distToPi > distToZero) ? Math.PI : 0;

        timer.reset();

        // Phase 3: Toggle target continuously to observe system settling and overshoot
        while (opModeIsActive()) {
            update();

            // Toggle target every 3 seconds
            if (timer.time(TimeUnit.MILLISECONDS) > 3000) {
                verificationTarget = (verificationTarget == Math.PI) ? 0 : Math.PI;
                timer.reset();
            }

            double error = AngleUnit.normalizeRadians(verificationTarget - localizer.getPose().getHeading());
            turnTo(error);

            telemetry.addData("Phase", "3/3: Final Verification");
            telemetry.addData("Status", "Tuning Complete. Observe behavior.");
            telemetry.addLine();
            telemetry.addData("Calculated kP", kP);
            telemetry.addData("Calculated kD", kD);
            telemetry.addData("Calculated kS", kS);
            telemetry.addLine();
            telemetry.addData("Target", verificationTarget);
            telemetry.addData("Current Heading", localizer.getPose().getHeading());
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // endregion
    }

    private void turnTo(double error) {
        drivetrain.moveWithVectors(0, 0, -controller.calculate(error));
    }

    private void update() {
        localizer.update();
    }
}