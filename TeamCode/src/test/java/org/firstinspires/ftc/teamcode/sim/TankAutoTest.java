package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import core.ApexConstants;
import org.firstinspires.ftc.teamcode.apexpathing.AutoTest;

/** Selects tank hardware; the route, execution, checks, and logging all run through AutoTest. */
@Autonomous(name = "Tank Apex Auto Test", group = "Apex Tank Simulation")
public final class TankAutoTest extends AutoTest {
    @Override protected boolean useTankPath() { return true; }

    @Override protected ApexConstants createConstants() {
        return new TankSimulationConstants();
    }

    @Override public void runOpMode() {
        if (!TankSimulationEnvironment.requireTankLauncher(this)) { return; }
        super.runOpMode();
    }
}
