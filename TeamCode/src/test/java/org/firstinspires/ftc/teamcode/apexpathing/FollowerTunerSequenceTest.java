package org.firstinspires.ftc.teamcode.apexpathing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FollowerTunerSequenceTest {
    @Test
    public void advancesThroughEveryTuningPhaseInOrder() {
        FollowerTuner.Phase[] phases = FollowerTuner.Phase.values();

        for (int i = 0; i < phases.length - 1; i++) {
            assertEquals(phases[i + 1], FollowerTuner.nextPhase(phases[i]));
        }
        assertNull(FollowerTuner.nextPhase(phases[phases.length - 1]));
    }

    @Test
    public void rampRunsBeforeControllersAndLimits() {
        assertArrayEquals(new FollowerTuner.Phase[] {
                        FollowerTuner.Phase.FEEDFORWARD,
                        FollowerTuner.Phase.HEADING,
                        FollowerTuner.Phase.DRIVE,
                        FollowerTuner.Phase.LIMITS,
                        FollowerTuner.Phase.ACCELERATION_FEEDFORWARD,
                        FollowerTuner.Phase.CENTRIPETAL,
                        FollowerTuner.Phase.VELOCITY_FEEDBACK
                },
                FollowerTuner.Phase.values());
        assertEquals(FollowerTuner.Phase.HEADING,
                FollowerTuner.nextPhase(FollowerTuner.Phase.FEEDFORWARD));
        assertEquals(FollowerTuner.Phase.LIMITS,
                FollowerTuner.nextPhase(FollowerTuner.Phase.DRIVE));
        assertEquals(FollowerTuner.Phase.ACCELERATION_FEEDFORWARD,
                FollowerTuner.nextPhase(FollowerTuner.Phase.LIMITS));
        assertEquals(FollowerTuner.Phase.CENTRIPETAL,
                FollowerTuner.nextPhase(FollowerTuner.Phase.ACCELERATION_FEEDFORWARD));
    }

    @Test
    public void centripetalCompletionAdvancesToVelocityFeedback() {
        assertEquals(FollowerTuner.Phase.VELOCITY_FEEDBACK,
                FollowerTuner.nextPhase(FollowerTuner.Phase.CENTRIPETAL));
    }

    @Test
    public void feedforwardCompletionDoesNotRequireAccelerationOrNonzeroFriction() throws Exception {
        core.FollowerConstants constants = core.FollowerConstants.fromJson(
                new org.json.JSONObject().put("drivetrainType", "MECANUM"));
        constants.angularKV = .1;
        constants.translationalKV = .01;
        constants.angularKA = constants.translationalKA = 0;
        constants.angularFeedforwardKS = constants.translationalFeedforwardKS = 0;
        assertTrue(FollowerTuner.Phase.FEEDFORWARD.isTunedPredicate.test(constants));
        constants.angularKV = 0;
        assertFalse(FollowerTuner.Phase.FEEDFORWARD.isTunedPredicate.test(constants));
    }

    @Test
    public void velocityFeedbackRequiresBothAxesToBeTuned() {
        assertFalse(FollowerTuner.velocityFeedbackTuned(0.0, 0.25));
        assertFalse(FollowerTuner.velocityFeedbackTuned(0.10, 0.0));
        assertTrue(FollowerTuner.velocityFeedbackTuned(0.10, 0.25));
    }

    @Test
    public void everyPhaseCanBeSelectedRegardlessOfPriorCompletion() {
        FollowerTuner.Phase[] phases = FollowerTuner.Phase.values();
        boolean[] original = new boolean[phases.length];
        for (int i = 0; i < phases.length; i++) {
            original[i] = phases[i].tuned;
            phases[i].tuned = false;
        }

        try {
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.HEADING));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.FEEDFORWARD));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.LIMITS));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.ACCELERATION_FEEDFORWARD));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.DRIVE));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.VELOCITY_FEEDBACK));

            FollowerTuner.Phase.HEADING.tuned = true;
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.HEADING));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.FEEDFORWARD));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.LIMITS));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.ACCELERATION_FEEDFORWARD));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.DRIVE));
        } finally {
            for (int i = 0; i < phases.length; i++) { phases[i].tuned = original[i]; }
        }
    }
}
