package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class FeedforwardTunerTest {
    @Test
    public void integratedWindowsRecoverFeedforwardAcrossAccelerationAndBraking() {
        double expectedKS = 0.08;
        double expectedKV = 0.02;
        double expectedKA = 0.004;
        double dt = 0.02;
        int samplesPerWindow = 10;
        double velocity = 0.0;
        List<FeedforwardTuner.IntegralObservation> windows = new ArrayList<>();

        // Each row is {power, sample count}. Lower positive and negative power produce powered
        // deceleration while the mechanism is still moving forward.
        double[][] stages = {
                { 0.55, 60 }, { 0.30, 25 }, { 0.12, 15 }, { -0.08, 8 },
                { 0.62, 45 }, { 0.22, 22 }, { 0.02, 10 }
        };
        double integratedPower = 0.0;
        double integratedSign = 0.0;
        double integratedPosition = 0.0;
        double windowStartVelocity = velocity;
        int windowSamples = 0;

        for (double[] stage : stages) {
            double power = stage[0];
            for (int sample = 0; sample < (int) stage[1]; sample++) {
                double sign = velocity > 1e-9 ? 1.0 : Math.signum(power);
                double acceleration = (power - expectedKS * sign - expectedKV * velocity) /
                        expectedKA;

                integratedPower += power * dt;
                integratedSign += sign * dt;
                integratedPosition += velocity * dt;
                velocity = Math.max(0.0, velocity + acceleration * dt);
                windowSamples++;

                if (windowSamples == samplesPerWindow) {
                    windows.add(new FeedforwardTuner.IntegralObservation(
                            integratedSign,
                            integratedPosition,
                            velocity - windowStartVelocity,
                            integratedPower,
                            velocity - windowStartVelocity < 0.0));
                    integratedPower = 0.0;
                    integratedSign = 0.0;
                    integratedPosition = 0.0;
                    windowStartVelocity = velocity;
                    windowSamples = 0;
                }
            }
        }

        FeedforwardTuner.IntegralFitResult fit =
                FeedforwardTuner.fitIntegralFeedforward(windows);

        assertTrue("Test must include powered deceleration",
                windows.stream().anyMatch(row -> row.velocityChange < -0.5));
        assertTrue(fit.brakingWindows > 0);
        assertEquals(expectedKS, fit.kS, 0.002);
        assertEquals(expectedKV, fit.kV, 0.0005);
        assertEquals(expectedKA, fit.kA, 0.0002);
    }

    @Test
    public void integralFitRejectsAccelerationWithoutBrakingEvidence() {
        List<FeedforwardTuner.IntegralObservation> windows = new ArrayList<>();
        for (int i = 1; i <= 16; i++) {
            double time = 0.2;
            double distance = 0.25 * i;
            double velocityChange = 0.4 + 0.03 * i;
            double powerTime = 0.08 * time + 0.02 * distance +
                    0.004 * velocityChange;
            windows.add(new FeedforwardTuner.IntegralObservation(
                    time, distance, velocityChange, powerTime, false));
        }

        FeedforwardTuner.IntegralFitResult fit =
                FeedforwardTuner.fitIntegralFeedforward(windows);

        assertEquals(0, fit.brakingWindows);
        assertTrue(!fit.isValid());
    }

    @Test
    public void robustRegressionRecoversFeedforwardFromAllFourRuns() {
        double kS = 0.08;
        double expectedKV = 0.025;
        double expectedKA = 0.004;
        List<FeedforwardTuner.Observation> samples = new ArrayList<>();

        for (int run = 0; run < 4; run++) {
            for (int i = 1; i <= 35; i++) {
                double velocity = 2.0 + i * 0.7 + run * 0.15;
                double acceleration = run < 2 ? 0.3 + i * 0.01 : 8.0 + i * 0.2;
                double noise = ((i % 5) - 2) * 0.0005;
                double power = kS + expectedKV * velocity + expectedKA * acceleration + noise;
                if (run == 2 && i == 17) { power += 0.35; }
                samples.add(new FeedforwardTuner.Observation(
                        power, velocity, acceleration, run));
            }
        }

        FeedforwardTuner.FitResult fit = FeedforwardTuner.fitFeedforward(samples, kS);

        assertTrue(fit.isValid());
        assertEquals(expectedKV, fit.kV, 0.001);
        assertEquals(expectedKA, fit.kA, 0.001);
        assertEquals(140, fit.sampleCount);
        assertTrue(fit.rSquared > 0.95);
    }

    @Test
    public void unrelatedVelocityAndAccelerationDoNotPassValidation() {
        double kS = 0.08;
        List<FeedforwardTuner.Observation> samples = new ArrayList<>();
        for (int run = 0; run < 4; run++) {
            for (int i = 1; i <= 30; i++) {
                double velocity = i * 0.5;
                double acceleration = ((i * 7 + run * 3) % 11) - 5.0;
                double power = 0.15 + ((i * 13 + run * 5) % 17) * 0.03;
                samples.add(new FeedforwardTuner.Observation(
                        power, velocity, acceleration, run));
            }
        }

        FeedforwardTuner.FitResult fit = FeedforwardTuner.fitFeedforward(samples, kS);

        assertTrue(!fit.isValid());
    }

    @Test
    public void manualAngularPowerAlternatesDirectionAndClampsSaturation() {
        double counterclockwise = FeedforwardTuner.manualAngularPower(
                0.2, 0.1, 0.15, 3.0, 2.0, 1.0);
        double clockwise = FeedforwardTuner.manualAngularPower(
                0.2, 0.1, 0.15, 3.0, 2.0, -1.0);

        assertEquals(-counterclockwise, clockwise, 1e-12);
        assertEquals(0.95, counterclockwise, 1e-12);
        assertEquals(1.0, FeedforwardTuner.clipManualPower(1.4), 0.0);
        assertEquals(-1.0, FeedforwardTuner.clipManualPower(-1.4), 0.0);
    }

}
