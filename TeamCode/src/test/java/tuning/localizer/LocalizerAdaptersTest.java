package tuning.localizer;

import org.json.JSONObject;
import org.junit.Test;

import java.util.Arrays;

import geometry.Angle;
import geometry.Dist;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import localizers.Pinpoint;
import localizers.MecanumDriveEncoders;
import localizers.Octoquad;
import localizers.OTOS;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import localizers.TankDriveEncoders;
import localizers.ThreeWheel;
import localizers.TwoWheel;

import static org.junit.Assert.assertEquals;

public class LocalizerAdaptersTest {
    private static final double EPS = 1e-6;

    @Test public void distanceScaleUsesPhysicalOverReportedRatio() {
        TwoWheel.Constants config = new TwoWheel.Constants().setTicksPerInch(100.0);
        LocalizerAdapter adapter = LocalizerAdapters.create(config);
        CalibrationCandidate result = adapter.fitDistance(config, CalibrationAxis.FORWARD,
                snapshot(0, 0, 0, 0, 0), snapshot(10, 0, 0, 1000, 0), 20.0);
        assertEquals(50.0, result.getValues().optDouble("ticksPerInch"), EPS);
    }

    @Test public void pairedThreeWheelSpinFindsSeparationAndPerpendicularOffset() {
        ThreeWheel.Constants config = new ThreeWheel.Constants()
                .setTicksPerInch(100.0).setOffsets(Vector.zero());
        LocalizerAdapter adapter = LocalizerAdapters.create(config);
        SpinTrial ccw = new SpinTrial(snapshot(0, 0, 0, 0, 0, 0),
                snapshot(0, 0, 4, 2000, -2000, 1200), 4.0, 4.0);
        SpinTrial cw = new SpinTrial(snapshot(0, 0, 0, 0, 0, 0),
                snapshot(0, 0, -4, -2000, 2000, -1200), -4.0, -4.0);
        JSONObject values = adapter.fitSpin(config, Arrays.asList(ccw, cw)).getValues();
        assertEquals(10.0, values.optDouble("xIn"), EPS);
        assertEquals(3.0, values.optDouble("yIn"), EPS);
    }

    @Test public void presetPinpointCalibrationIsSavedInDistStorageUnits() {
        Pinpoint.Constants config = new Pinpoint.Constants()
                .setEncoderResolution(Pinpoint.GoBildaPods.goBILDA_4_BAR_POD);
        LocalizerAdapter adapter = LocalizerAdapters.create(config);
        JSONObject result = adapter.fitDistance(config, CalibrationAxis.FORWARD,
                snapshot(0, 0, 0), snapshot(10, 0, 0), 20).getValues();
        assertEquals(19.89436789 / 25.4 * 0.5,
                result.optDouble("customEncoderResolutionIn"), EPS);
    }

    @Test public void capabilityMatrixCoversEveryBuiltInAndCustomLocalizers() {
        TankDriveEncoders.Constants tank = new TankDriveEncoders.Constants()
                .addLeftEncoder("left", 100, false)
                .addRightEncoder("right", 100, false);
        assertCapabilities(tank, true, false, false);

        MecanumDriveEncoders.Constants twoMotorMecanum = new MecanumDriveEncoders.Constants();
        twoMotorMecanum.frontLeftName = "left";
        twoMotorMecanum.frontRightName = "right";
        assertCapabilities(twoMotorMecanum, true, false, false);
        twoMotorMecanum.backLeftName = "backLeft";
        twoMotorMecanum.backRightName = "backRight";
        assertCapabilities(twoMotorMecanum, true, true, false);

        assertCapabilities(new TwoWheel.Constants(), true, true, true);
        assertCapabilities(new ThreeWheel.Constants(), true, true, true);
        assertCapabilities(new Pinpoint.Constants(), true, true, true);
        assertCapabilities(new Octoquad.Constants(), true, true, true);
        assertCapabilities(new OTOS.Constants(), true, true, true);
        assertCapabilities(new CustomConfig(), false, false, false);
    }

    @Test public void tankDistanceFitTunesEveryMotorOnBothSides() {
        TankDriveEncoders.Constants config = new TankDriveEncoders.Constants()
                .addLeftEncoder("leftFront", 100, false)
                .addLeftEncoder("leftBack", 200, false)
                .addRightEncoder("rightFront", 100, true)
                .addRightEncoder("rightBack", 200, true);
        JSONObject values = LocalizerAdapters.create(config).fitDistance(config,
                CalibrationAxis.FORWARD, snapshot(0, 0, 0, 0, 0, 0, 0),
                snapshot(10, 0, 0, 1000, 2000, -1000, -2000), 20).getValues();
        assertEquals(50.0, values.optJSONArray("left").optJSONObject(0)
                .optDouble("ticksPerInch"), EPS);
        assertEquals(100.0, values.optJSONArray("left").optJSONObject(1)
                .optDouble("ticksPerInch"), EPS);
        assertEquals(50.0, values.optJSONArray("right").optJSONObject(0)
                .optDouble("ticksPerInch"), EPS);
        assertEquals(100.0, values.optJSONArray("right").optJSONObject(1)
                .optDouble("ticksPerInch"), EPS);
    }

    @Test public void remainingBuiltInFitsProduceExpectedScalesAndOffsets() {
        MecanumDriveEncoders.Constants mecanum = new MecanumDriveEncoders.Constants()
                .setTicksPerInch(100.0);
        JSONObject mecanumValues = LocalizerAdapters.create(mecanum).fitDistance(mecanum,
                CalibrationAxis.STRAFE, snapshot(0, 0, 0), snapshot(0, 10, 0), 20)
                .getValues();
        assertEquals(50.0, mecanumValues.optDouble("ticksPerInch"), EPS);

        Octoquad.Constants octoquad = new Octoquad.Constants()
                .setEncoderResolution(Dist.fromMm(100.0 / 25.4))
                .setOffsets(1.0, 2.0, DistUnit.IN)
                .setYawScalar(0.8);
        JSONObject octoDistance = LocalizerAdapters.create(octoquad).fitDistance(octoquad,
                CalibrationAxis.FORWARD, snapshot(0, 0, 0), snapshot(10, 0, 0), 20)
                .getValues();
        assertEquals(octoquad.encoderResolution.getIn() / 2.0,
                octoDistance.optDouble("encoderResolutionIn"), EPS);
        JSONObject octoSpin = LocalizerAdapters.create(octoquad).fitSpin(octoquad,
                pairedComputerSpins()).getValues();
        assertEquals(5.0, octoSpin.optDouble("xIn"), EPS);
        assertEquals(-3.0, octoSpin.optDouble("yIn"), EPS);
        assertEquals(1.6, octoSpin.optDouble("angularScalar"), EPS);

        Pinpoint.Constants pinpoint = new Pinpoint.Constants()
                .setEncoderResolution(Dist.fromMm(100.0 / 25.4))
                .setOffsets(1.0, 2.0, DistUnit.IN)
                .setAngularScalar(0.8);
        JSONObject pinpointSpin = LocalizerAdapters.create(pinpoint).fitSpin(pinpoint,
                pairedComputerSpins()).getValues();
        assertEquals(5.0, pinpointSpin.optDouble("xIn"), EPS);
        assertEquals(-3.0, pinpointSpin.optDouble("yIn"), EPS);
        assertEquals(1.6, pinpointSpin.optDouble("angularScalar"), EPS);

        OTOS.Constants otos = new OTOS.Constants();
        JSONObject otosDistance = LocalizerAdapters.create(otos).fitDistance(otos,
                CalibrationAxis.FORWARD, snapshot(0, 0, 0), snapshot(10, 0, 0), 11)
                .getValues();
        assertEquals(1.1, otosDistance.optDouble("linearScalar"), EPS);
        JSONObject otosSpin = LocalizerAdapters.create(otos).fitSpin(otos, Arrays.asList(
                new SpinTrial(snapshot(0, 0, 0), snapshot(0, 0, 3.6), 4.0, 3.6),
                new SpinTrial(snapshot(0, 0, 0), snapshot(0, 0, -3.6), -4.0, -3.6)))
                .getValues();
        assertEquals(8.0 / 7.2, otosSpin.optDouble("angularScalar"), EPS);
    }

    private static java.util.List<SpinTrial> pairedComputerSpins() {
        return Arrays.asList(
                new SpinTrial(snapshot(0, 0, 0, 0, 0),
                        snapshot(0, 0, 2, 2000, -1200), 4.0, 2.0),
                new SpinTrial(snapshot(0, 0, 0, 0, 0),
                        snapshot(0, 0, -2, -2000, 1200), -4.0, -2.0));
    }

    private static void assertCapabilities(BaseLocalizerConstants<?> config, boolean forward,
                                           boolean strafe, boolean spin) {
        LocalizerAdapter adapter = LocalizerAdapters.create(config);
        assertEquals(forward, adapter.supportsDistance(CalibrationAxis.FORWARD));
        assertEquals(strafe, adapter.supportsDistance(CalibrationAxis.STRAFE));
        assertEquals(spin, adapter.supportsSpinCalibration());
    }

    private static final class CustomConfig implements BaseLocalizerConstants<CustomConfig> {
        @Override public BaseLocalizer<?> build(
                com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
            throw new UnsupportedOperationException();
        }
    }

    private static CalibrationSnapshot snapshot(double x, double y, double heading, int... ticks) {
        return new CalibrationSnapshot(new Pose(Vector.of(x, y, DistUnit.IN),
                Angle.fromRad(heading)), ticks);
    }
}
