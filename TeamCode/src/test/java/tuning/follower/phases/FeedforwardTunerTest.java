package tuning.follower.phases;

import org.junit.Test;
import static org.junit.Assert.*;

public class FeedforwardTunerTest {
    @Test public void recoversDriveAndTurnCoefficientsFromMovingSamples() {
        for (double kV : new double[] { .012, .12 }) {
            RampRegression regression = new RampRegression(.1);
            for (int i = 1; i <= 60; i++) {
                double power = .15 + i * .01;
                regression.add((power - .15) / kV, power);
            }
            RampRegression.Fit fit = regression.fit();
            assertTrue(fit.valid(.2));
            assertEquals(.15, fit.kS, 1e-10);
            assertEquals(kV, fit.kV, 1e-10);
            assertEquals(1, fit.rSquared, 1e-10);
        }
    }

    @Test public void stationaryReverseInvalidAndSaturatedSamplesCannotContaminateFit() {
        RampRegression regression = new RampRegression(1);
        assertFalse(regression.add(0, .1));
        assertFalse(regression.add(-2, .2));
        assertFalse(regression.add(Double.NaN, .2));
        assertFalse(regression.add(2, Double.POSITIVE_INFINITY));
        assertFalse(regression.add(2, 1.1));
        assertFalse(regression.fit().valid(2));
    }

    @Test public void insufficientOrConstantVelocityDataCannotBeAccepted() {
        RampRegression regression = new RampRegression(.1);
        for (int i = 0; i < 40; i++) { regression.add(2, .2 + i * .01); }
        assertFalse(regression.fit().valid(.2));
        regression = new RampRegression(.1);
        for (int i = 1; i < 10; i++) { regression.add(i, .1 + .02 * i); }
        assertFalse(regression.fit().valid(.2));
    }

    @Test public void operatorCanExcludeAndRestoreAnOutlier() {
        RampRegression regression = new RampRegression(.1);
        for (int i = 1; i <= 40; i++) { regression.add(i, .1 + .01 * i); }
        regression.add(20, .85);
        double contaminated = regression.fit().kS;
        assertTrue(regression.excludeWorst());
        assertEquals(.1, regression.fit().kS, 1e-10);
        assertEquals(.01, regression.fit().kV, 1e-10);
        regression.restore();
        assertEquals(contaminated, regression.fit().kS, 1e-10);
    }

    @Test public void negativeSlopeOrInterceptIsRejectedAndZeroFrictionIsAllowed() {
        RampRegression negativeSlope = new RampRegression(.1);
        RampRegression negativeIntercept = new RampRegression(.1);
        RampRegression zeroFriction = new RampRegression(.1);
        for (int i = 1; i <= 40; i++) {
            negativeSlope.add(i, .8 - .01 * i);
            negativeIntercept.add(i + 10, .01 * i);
            zeroFriction.add(i, .01 * i);
        }
        assertFalse(negativeSlope.fit().valid(2));
        assertFalse(negativeIntercept.fit().valid(2));
        assertTrue(zeroFriction.fit().valid(2));
    }
}
