package localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.json.JSONArray;
import org.json.JSONObject;

import geometry.Pose;

/**
 * Tank odometry using one or more drive encoders per side and IMU heading.
 * Each side is averaged independently, so unequal encoder counts do not bias translation.
 * X is forward at zero heading; Y is left. Heading is continuous across IMU wraparound.
 * Assumes rolling without lateral slip and approximately constant curvature per update.
 * Motor directions and run modes are never changed, including during pose resets.
 */
public class TankDriveEncoders extends BaseLocalizer<TankDriveEncoders.Constants> {
    private final IntSupplier[] encoders;
    private final double[] inchesPerTick;
    private final int[] previousTicks;
    private final int leftCount;
    private final DoubleSupplier yaw;
    private double previousYaw;

    public TankDriveEncoders(Constants constants, HardwareMap hardwareMap) {
        this(constants, connect(constants, hardwareMap));
    }

    private TankDriveEncoders(Constants constants, Sensors sensors) {
        this(constants, sensors.encoders, sensors.yaw);
    }

    // Sensor seam for deterministic odometry tests without Android hardware.
    TankDriveEncoders(Constants constants, IntSupplier[] encoders, DoubleSupplier yaw) {
        super(constants);
        constants.validateEncoders();
        leftCount = constants.leftEncoders.size();
        List<Encoder> specs = new ArrayList<>(constants.leftEncoders);
        specs.addAll(constants.rightEncoders);
        if (encoders.length != specs.size()) {
            throw new IllegalArgumentException("Encoder input count does not match configuration");
        }
        this.encoders = encoders.clone();
        this.yaw = yaw;
        previousTicks = new int[encoders.length];
        inchesPerTick = new double[encoders.length];
        for (int i = 0; i < specs.size(); i++) {
            Encoder spec = specs.get(i);
            inchesPerTick[i] = (spec.reversed ? -1.0 : 1.0) / spec.ticksPerInch;
        }
        setPose(Pose.zero());
    }

    @Override
    public void update() {
        double currentYaw = yaw.getAsDouble();
        if (!Double.isFinite(currentYaw)) { return; }
        double left = 0.0;
        double right = 0.0;
        for (int i = 0; i < encoders.length; i++) {
            int ticks = encoders[i].getAsInt();
            // Integer subtraction also handles signed encoder rollover.
            double distance = (ticks - previousTicks[i]) * inchesPerTick[i];
            previousTicks[i] = ticks;
            if (i < leftCount) { left += distance; }
            else { right += distance; }
        }
        double distance = 0.5 * (left / leftCount + right / (encoders.length - leftCount));
        double turn = Math.atan2(Math.sin(currentYaw - previousYaw),
                Math.cos(currentYaw - previousYaw));
        previousYaw = currentYaw;
        double heading = pose.getHeading().getRad();
        pose = integrateArc(pose, distance, 0.0, turn, heading + turn);
        calculate(UpdateType.BOTH);
    }

    /** Returns raw encoder positions with all configured left motors followed by right motors. */
    public int[] getEncoderTicks() { return previousTicks.clone(); }

    @Override
    public void setPose(Pose newPose) {
        double currentYaw = yaw.getAsDouble();
        if (!Double.isFinite(currentYaw)) {
            throw new IllegalStateException("Cannot reset tank localization with invalid IMU yaw");
        }
        for (int i = 0; i < encoders.length; i++) {
            previousTicks[i] = encoders[i].getAsInt();
        }
        previousYaw = currentYaw;
        resetKinematicEstimate(newPose);
        // Prime derivative history so the first movement update is included.
        calculate(UpdateType.BOTH);
    }

    private static Sensors connect(Constants constants, HardwareMap hardwareMap) {
        constants.validateEncoders();
        if (constants.imuName == null || constants.imuName.trim().isEmpty()
                || constants.hubOrientation == null) {
            throw new IllegalArgumentException("Set the IMU name and hub orientation");
        }
        List<Encoder> specs = new ArrayList<>(constants.leftEncoders);
        specs.addAll(constants.rightEncoders);
        IntSupplier[] inputs = new IntSupplier[specs.size()];
        for (int i = 0; i < specs.size(); i++) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, specs.get(i).name);
            inputs[i] = motor::getCurrentPosition;
        }
        IMU imu = hardwareMap.get(IMU.class, constants.imuName);
        if (!imu.initialize(new IMU.Parameters(constants.hubOrientation))) {
            throw new IllegalStateException("Tank localizer IMU initialization failed");
        }
        return new Sensors(inputs, () -> imu.getRobotYawPitchRollAngles()
                .getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS));
    }

    private static final class Sensors {
        final IntSupplier[] encoders;
        final DoubleSupplier yaw;
        Sensors(IntSupplier[] encoders, DoubleSupplier yaw) {
            this.encoders = encoders;
            this.yaw = yaw;
        }
    }

    public static final class Encoder {
        public final String name;
        public final double ticksPerInch;
        public final boolean reversed;

        /** Reversal is relative to the SDK reading, which already reflects motor direction. */
        public Encoder(String name, double ticksPerInch, boolean reversed) {
            this.name = name;
            this.ticksPerInch = ticksPerInch;
            this.reversed = reversed;
        }
    }

    public static class Constants implements BaseLocalizerConstants<Constants> {
        public final List<Encoder> leftEncoders = new ArrayList<>();
        public final List<Encoder> rightEncoders = new ArrayList<>();
        public String imuName;
        public RevHubOrientationOnRobot hubOrientation;

        @Override
        public JSONObject getCalibrationValues() {
            try {
                JSONObject result = new JSONObject();
                result.put("left", encoders(leftEncoders));
                result.put("right", encoders(rightEncoders));
                return result;
            } catch (Exception e) { throw new IllegalStateException(e); }
        }

        @Override
        public void applyCalibrationValues(JSONObject values) {
            applyEncoders(values.optJSONArray("left"), leftEncoders);
            applyEncoders(values.optJSONArray("right"), rightEncoders);
            validateEncoders();
        }

        private static JSONArray encoders(List<Encoder> encoders) throws Exception {
            JSONArray result = new JSONArray();
            for (Encoder encoder : encoders) {
                result.put(new JSONObject().put("name", encoder.name)
                        .put("ticksPerInch", encoder.ticksPerInch));
            }
            return result;
        }

        private static void applyEncoders(JSONArray saved, List<Encoder> target) {
            if (saved == null) { return; }
            for (int i = 0; i < saved.length(); i++) {
                JSONObject value = saved.optJSONObject(i);
                if (value == null) { continue; }
                String name = value.optString("name", "");
                for (int j = 0; j < target.size(); j++) {
                    Encoder current = target.get(j);
                    if (current.name.equals(name)) {
                        target.set(j, new Encoder(current.name, CalibrationJson.positive(value,
                                "ticksPerInch", current.ticksPerInch), current.reversed));
                    }
                }
            }
        }

        public Constants addLeftEncoder(String name, double ticksPerInch, boolean reversed) {
            leftEncoders.add(new Encoder(name, ticksPerInch, reversed));
            return this;
        }

        public Constants addRightEncoder(String name, double ticksPerInch, boolean reversed) {
            rightEncoders.add(new Encoder(name, ticksPerInch, reversed));
            return this;
        }

        public Constants setIMUName(String name) { imuName = name; return this; }

        public Constants setHubOrientation(RevHubOrientationOnRobot orientation) {
            hubOrientation = orientation;
            return this;
        }

        @Override
        public TankDriveEncoders build(HardwareMap hardwareMap) {
            return new TankDriveEncoders(this, hardwareMap);
        }

        private void validateEncoders() {
            if (leftEncoders.isEmpty() || rightEncoders.isEmpty()) {
                throw new IllegalArgumentException("Tank localization needs at least one encoder per side");
            }
            Set<String> names = new HashSet<>();
            List<Encoder> specs = new ArrayList<>(leftEncoders);
            specs.addAll(rightEncoders);
            for (Encoder spec : specs) {
                if (spec == null || spec.name == null || spec.name.trim().isEmpty()
                        || !Double.isFinite(spec.ticksPerInch) || spec.ticksPerInch <= 0
                        || !names.add(spec.name)) {
                    throw new IllegalArgumentException("Encoders need unique names and finite positive ticksPerInch");
                }
            }
        }
    }
}
