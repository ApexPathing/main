package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Prevents tank-only OpModes from running against the normal mecanum simulator launcher. */
final class TankSimulationEnvironment {
    private TankSimulationEnvironment() { }

    static boolean requireTankLauncher(LinearOpMode opMode) {
        if (ApexSimulation.isTankSimulation()) { return true; }
        opMode.telemetry.addLine("This OpMode requires tank-drive simulation physics.");
        opMode.telemetry.addLine("Run the SimulateTankApexPathing JUnit test instead.");
        opMode.telemetry.update();
        return false;
    }
}
