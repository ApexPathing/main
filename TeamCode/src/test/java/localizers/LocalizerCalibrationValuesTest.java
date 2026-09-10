package localizers;

import org.json.JSONObject;
import org.junit.Test;

import geometry.Angle;
import geometry.Dist;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import tuning.localizer.CalibrationAxis;
import tuning.localizer.LocalizerAdapter;
import tuning.localizer.LocalizerAdapters;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class LocalizerCalibrationValuesTest {
    @Test public void everyBuiltInLocalizerHasSerializableCalibrationValues() {
        assertRoundTrip(new TankDriveEncoders.Constants()
                .addLeftEncoder("l1", 100, false).addLeftEncoder("l2", 110, true)
                .addRightEncoder("r1", 105, false));
        assertRoundTrip(new MecanumDriveEncoders.Constants().setTicksPerInch(95));
        assertRoundTrip(new TwoWheel.Constants().setTicksPerInch(101)
                .setOffsets(Vector.of(2, 3, DistUnit.IN)));
        assertRoundTrip(new ThreeWheel.Constants().setTicksPerInch(102)
                .setOffsets(Vector.of(11, -4, DistUnit.IN)));
        assertRoundTrip(new Pinpoint.Constants().setOffsets(2, -3, DistUnit.IN)
                .setEncoderResolution(Dist.of(19.8, DistUnit.MM)).setAngularScalar(1.01));
        assertRoundTrip(new Octoquad.Constants().setOffsets(1, 2, DistUnit.IN)
                .setEncoderResolution(Dist.of(20.1, DistUnit.MM)).setYawScalar(0.99));
        assertRoundTrip(new OTOS.Constants().setOffset(new Pose(
                Vector.of(1, 2, DistUnit.IN), Angle.fromRad(0.1)))
                .setLinearScalar(1.01).setAngularScalar(0.99));
    }

    @Test public void adapterCapabilitiesMatchLocalizerAndEncoderLayout() {
        LocalizerAdapter twoEncoderMecanum = LocalizerAdapters.create(
                new MecanumDriveEncoders.Constants().setFrontLeftName("l").setFrontRightName("r"));
        assertTrue(twoEncoderMecanum.supportsDistance(CalibrationAxis.FORWARD));
        assertFalse(twoEncoderMecanum.supportsDistance(CalibrationAxis.STRAFE));
        assertFalse(LocalizerAdapters.create(new TankDriveEncoders.Constants())
                .supportsDistance(CalibrationAxis.STRAFE));
        assertTrue(LocalizerAdapters.create(new ThreeWheel.Constants()).supportsSpinCalibration());
        assertTrue(LocalizerAdapters.create(new Pinpoint.Constants()).supportsSpinCalibration());
        assertTrue(LocalizerAdapters.create(new Octoquad.Constants()).supportsSpinCalibration());
        assertTrue(LocalizerAdapters.create(new OTOS.Constants()).supportsSpinCalibration());
    }

    private static void assertRoundTrip(BaseLocalizerConstants<?> config) {
        JSONObject before = config.getCalibrationValues();
        config.applyCalibrationValues(before);
        assertEquals(before.toString(), config.getCalibrationValues().toString());
        assertFalse(config.getCalibrationType().isEmpty());
    }
}
