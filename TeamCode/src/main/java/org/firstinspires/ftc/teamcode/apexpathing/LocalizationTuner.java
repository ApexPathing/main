package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Function;

import core.ApexConstants;
import core.LocalizationConstants;
import drivetrains.DualActuated;
import tuning.localizer.CalibrationAxis;
import tuning.localizer.DistancePhase;
import tuning.localizer.FilterPhase;
import tuning.localizer.HardwarePhase;
import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.SpinPhase;
import tuning.localizer.TuningPhase;
import tuning.localizer.ValidationPhase;

/**
 * Tunes localizer geometry and derivative filtering independently from follower control gains.
 * Built-in localizer differences are isolated behind calibration adapters.
 */
@TeleOp(name = "Localization Tuner", group = "Apex Pathing")
public class LocalizationTuner extends LinearOpMode {
    private enum Phase {
        HARDWARE(HardwarePhase::new),
        FORWARD(context -> new DistancePhase(context, CalibrationAxis.FORWARD)),
        STRAFE(context -> new DistancePhase(context, CalibrationAxis.STRAFE)),
        ROTATION(SpinPhase::new),
        FILTER(FilterPhase::new),
        VALIDATE(ValidationPhase::new);

        final Function<LocalizationTunerContext, TuningPhase> factory;
        Phase(Function<LocalizationTunerContext, TuningPhase> factory) { this.factory = factory; }
    }

    private LocalizationTunerContext context;
    private int selected;
    private boolean selectedPhase;

    @Override public void runOpMode() {
        selected = 0;
        selectedPhase = false;
        context = new LocalizationTunerContext(this, createConstants());

        while (opModeInInit() && !selectedPhase) {
            context.updateDebugMode(true);
            selectPhase();
            sleep(20);
        }
        while (opModeInInit()) {
            telemetry.clearAll();
            context.addInterfaceHeader();
            telemetry.addLine("Press Start to run the selected phase.");
            telemetry.update();
            sleep(20);
        }
        while (opModeIsActive()) {
            if (!selectedPhase) {
                selectPhase();
                sleep(20);
                continue;
            }
            Phase current = Phase.values()[selected];
            boolean accepted = current.factory.apply(context).run(this);
            selectedPhase = false;
            if (accepted) { selectNextApplicable(); }
        }
        context.stop();
    }

    /**
     * Supplies the robot configuration used by the tuner.
     *
     * <p>The hook keeps the installed OpMode on the team's {@link Constants} while allowing
     * simulator tests and downstream projects to exercise the same UI with another supported
     * drivetrain/localizer pairing.</p>
     */
    protected ApexConstants createConstants() { return new Constants(); }

    private void selectPhase() {
        context.update();
        telemetry.clearAll();
        context.addInterfaceHeader();
        if (context.getDrivetrain() instanceof DualActuated) {
            telemetry.addLine("Dpad Left/Right: switch drivetrain mode.");
            if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
                context.toggleDualActuatedMode();
                selectFirstApplicable();
            }
        }
        telemetry.addLine("Select a localization phase");
        telemetry.addLine("Dpad Up/Down: choose   A: select");
        telemetry.addLine("Completed geometry and filters can be retuned independently.");
        telemetry.addLine();
        Phase[] phases = Phase.values();
        for (int i = 0; i < phases.length; i++) {
            telemetry.addLine(status(phases[i]) + " " + display(phases[i])
                    + (i == selected ? " <" : ""));
        }
        telemetry.update();

        if (gamepad1.dpadUpWasPressed()) {
            selected = (selected - 1 + phases.length) % phases.length;
        } else if (gamepad1.dpadDownWasPressed()) {
            selected = (selected + 1) % phases.length;
        } else if (gamepad1.aWasPressed() && applicable(phases[selected])
                && !context.isDriveTransitioning()) {
            selectedPhase = true;
        }
    }

    private boolean applicable(Phase phase) {
        if (phase == Phase.FORWARD) {
            return context.getAdapter().supportsDistance(CalibrationAxis.FORWARD);
        }
        if (phase == Phase.STRAFE) {
            return context.getDrivetrain().isHolonomic()
                    && context.getAdapter().supportsDistance(CalibrationAxis.STRAFE);
        }
        if (phase == Phase.ROTATION) { return context.getAdapter().supportsSpinCalibration(); }
        return true;
    }

    private String status(Phase phase) {
        if (!applicable(phase)) { return "[N/A]"; }
        LocalizationConstants calibration = context.getCalibration();
        if (phase == Phase.FORWARD || phase == Phase.STRAFE || phase == Phase.ROTATION) {
            String step = phase == Phase.FORWARD ? "FORWARD"
                    : phase == Phase.STRAFE ? "STRAFE" : "ROTATION";
            return "[" + shortStatus(calibration.getGeometryStepStatus(
                    context.getSetupName(), step)) + "]";
        }
        if (phase == Phase.FILTER) {
            return "[" + shortStatus(calibration.getStatus(context.getSetupName(), true)) + "]";
        }
        return "[READY]";
    }

    private static String shortStatus(LocalizationConstants.Status status) {
        if (status == LocalizationConstants.Status.ACCEPTED) { return "DONE"; }
        if (status == LocalizationConstants.Status.NEEDS_VALIDATION) { return "CHECK"; }
        return "READY";
    }

    private void selectNextApplicable() {
        Phase[] phases = Phase.values();
        for (int count = 0; count < phases.length; count++) {
            selected = (selected + 1) % phases.length;
            if (applicable(phases[selected])) { return; }
        }
    }

    private void selectFirstApplicable() {
        selected = 0;
        if (!applicable(Phase.values()[selected])) { selectNextApplicable(); }
    }

    private static String display(Phase phase) {
        if (phase == Phase.FORWARD) { return "FORWARD DISTANCE"; }
        if (phase == Phase.STRAFE) { return "STRAFE DISTANCE"; }
        return phase.name();
    }
}
