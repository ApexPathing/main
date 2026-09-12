package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.ApexConstants;
import org.firstinspires.ftc.teamcode.apexpathing.LocalizationTuner;

/** Localization tuner bound to the tank-physics Code Sim configuration. */
@TeleOp(name = "Tank Localization Tuner", group = "Apex Tank Simulation")
public final class TankLocalizationTuner extends LocalizationTuner {
    @Override public void runOpMode() {
        if (!TankSimulationEnvironment.requireTankLauncher(this)) { return; }
        super.runOpMode();
    }

    @Override protected ApexConstants createConstants() {
        return new TankSimulationConstants();
    }
}
