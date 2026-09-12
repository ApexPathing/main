package org.firstinspires.ftc.teamcode.sim;

import org.codeblooded.ftcodesim.simulator.FTCodeSimLinearOpModeRunner;
import org.junit.Assume;
import org.junit.Test;

import java.io.File;

import core.ApexStorage;

/** Interactive tank-physics Code Sim entry point. */
public class SimulateTankApexPathing {
    /** Set true only when intentionally launching the interactive simulator window. */
    private static final boolean RUN_INTERACTIVE_SIMULATOR = true;

    @Test public void tuneAndTestTankPathFollowing() throws Exception {
        Assume.assumeTrue("Set RUN_INTERACTIVE_SIMULATOR to true to launch tank Code Sim",
                RUN_INTERACTIVE_SIMULATOR);
        File directory = new File(System.getProperty("user.dir"), "build/ftcodesim-tank-data");
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        FTCodeSimLinearOpModeRunner.run(ApexSimulation.createTankSimulator());
    }
}
