package feedforward;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class FFLutTimeTest {
    @Test
    public void interpolatesPositionAndKinematicsByElapsedTime() {
        MotionParameters start = new MotionParameters(0.0, 0.0, 0.0, 4.0, 0.0)
                .setTimeSeconds(0.0);
        MotionParameters end = new MotionParameters(0.0, 0.0, 2.0, 0.0, 1.0)
                .setTimeSeconds(0.5);
        start.setMotorPower(0.2);
        end.setMotorPower(0.8);
        FFLut lut = new FFLut(new MotionParameters[]{start, end});

        MotionParameters midpoint = lut.getFFParamsByTime(0.25);
        assertEquals(0.5, midpoint.getDistAlongCurve(), 1e-9);
        assertEquals(1.0, midpoint.getAngularVel(), 1e-9);
        assertEquals(2.0, midpoint.getAngularAccel(), 1e-9);
        assertEquals(0.25, midpoint.getTimeSeconds(), 1e-9);
        assertEquals(0.5, midpoint.getMotorPower(), 1e-9);
        assertEquals(0.5, lut.getFFParams(0.5).getMotorPower(), 1e-9);
        assertEquals(0.5, lut.getDurationSeconds(), 1e-9);
    }

    @Test
    public void timeQueriesClampToTrajectoryEndpoints() {
        MotionParameters start = new MotionParameters().setTimeSeconds(0.0);
        MotionParameters end = new MotionParameters(0.0, 0.0, 0.0, 0.0, 2.0)
                .setTimeSeconds(1.0);
        FFLut lut = new FFLut(new MotionParameters[]{start, end});

        assertEquals(0.0, lut.getFFParamsByTime(-1.0).getDistAlongCurve(), 1e-9);
        assertEquals(2.0, lut.getFFParamsByTime(5.0).getDistAlongCurve(), 1e-9);
    }
}
