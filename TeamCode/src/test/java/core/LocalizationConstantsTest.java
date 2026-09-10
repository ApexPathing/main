package core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONObject;
import org.junit.Test;

import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class LocalizationConstantsTest {
    @Test public void capturesIndependentGeometryStepsAndRestoresSnapshot() {
        LocalizationConstants constants = LocalizationConstants.empty();
        TestConfig config = new TestConfig();
        TestLocalizer localizer = new TestLocalizer(config);
        localizer.setVelocityFilterMode(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE);
        localizer.setFilterWindow(9);
        constants.capture("DEFAULT", config, localizer,
                LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.NEEDS_VALIDATION);
        constants.markGeometryStepAccepted("DEFAULT", "FORWARD");
        JSONObject snapshot = constants.toJson();

        constants.markGeometryStepAccepted("DEFAULT", "ROTATION");
        assertEquals(LocalizationConstants.Status.ACCEPTED,
                constants.getGeometryStepStatus("DEFAULT", "ROTATION"));
        constants.restore(snapshot);
        assertEquals(LocalizationConstants.Status.UNCALIBRATED,
                constants.getGeometryStepStatus("DEFAULT", "ROTATION"));
        assertEquals(LocalizationConstants.Status.ACCEPTED,
                constants.getGeometryStepStatus("DEFAULT", "FORWARD"));

        TestConfig loaded = new TestConfig();
        loaded.scale = 1.0;
        assertTrue(constants.applyGeometry("DEFAULT", loaded));
        assertEquals(4.0, loaded.scale, 0.0);
        TestLocalizer filtered = new TestLocalizer(loaded);
        assertTrue(constants.applyFilter("DEFAULT", loaded, filtered));
        assertEquals(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE,
                filtered.getVelocityFilterMode());
        assertEquals(9, filtered.getFilterWindowSize());
    }

    @Test public void rejectsGeometryForDifferentLocalizerType() {
        LocalizationConstants constants = LocalizationConstants.empty();
        TestConfig config = new TestConfig();
        constants.capture("DEFAULT", config, new TestLocalizer(config),
                LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.ACCEPTED);
        assertFalse(constants.applyGeometry("DEFAULT", new OtherConfig()));
    }

    private static class TestConfig implements BaseLocalizerConstants<TestConfig> {
        double scale = 4.0;
        @Override public JSONObject getCalibrationValues() {
            try { return new JSONObject().put("scale", scale); }
            catch (Exception e) { throw new IllegalStateException(e); }
        }
        @Override public void applyCalibrationValues(JSONObject values) {
            scale = values.optDouble("scale", scale);
        }
        @Override public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            return new TestLocalizer(this);
        }
    }

    private static final class OtherConfig extends TestConfig { }

    private static final class TestLocalizer extends BaseLocalizer<TestConfig> {
        TestLocalizer(TestConfig config) { super(config); }
        @Override public void update() { calculate(UpdateType.BOTH); }
        @Override public void setPose(geometry.Pose pose) { resetKinematicEstimate(pose); }
    }
}
