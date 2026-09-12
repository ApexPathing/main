package feedforward;

import org.junit.Test;
import static org.junit.Assert.*;

public class FFLutDistanceTest {
    @Test public void brakingIntoRestPreservesConstantAcceleration() {
        FFLut lut = new FFLut(new MotionParameters[] {
                new MotionParameters(32, -128, -4, 16, 0),
                new MotionParameters(0, 0, 0, 0, 4)
        });
        MotionParameters midpoint = lut.getTankFFParams(2);
        assertEquals(Math.sqrt(512), midpoint.getTangentialVel(), 1e-10);
        assertEquals(-128, midpoint.getTangentialAccel(), 1e-10);
        assertEquals(-midpoint.getTangentialVel()/8, midpoint.getAngularVel(), 1e-10);
        assertEquals(16, midpoint.getAngularAccel(), 1e-10);
        double before = lut.getTankFFParams(2-.001).getTangentialVel();
        double after = lut.getTankFFParams(2+.001).getTangentialVel();
        assertEquals(midpoint.getTangentialAccel(), (after*after-before*before)/.004, 1e-8);
        assertEquals(0, lut.getTankFFParams(4).getTangentialAccel(), 0);
    }

    @Test public void interiorAccelerationInterpolationRetainsExistingBehavior() {
        FFLut lut = new FFLut(new MotionParameters[] {
                new MotionParameters(0, 50, 0, 5, 0),
                new MotionParameters(20, 0, 2, 0, 4)
        });
        MotionParameters sample = lut.getTankFFParams(1);
        assertEquals(5, sample.getTangentialVel(), 1e-10);
        assertEquals(37.5, sample.getTangentialAccel(), 1e-10);
        assertEquals(.5, sample.getAngularVel(), 1e-10);
        assertEquals(3.75, sample.getAngularAccel(), 1e-10);
    }
}
