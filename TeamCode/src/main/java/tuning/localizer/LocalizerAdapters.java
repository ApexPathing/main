package tuning.localizer;

import tuning.localizer.phases.*;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import localizers.MecanumDriveEncoders;
import localizers.Octoquad;
import localizers.OTOS;
import localizers.Pinpoint;
import localizers.TankDriveEncoders;
import localizers.ThreeWheel;
import localizers.TwoWheel;

/** Factory and compact built-in implementations for localization calibration adapters. */
public final class LocalizerAdapters {
    private LocalizerAdapters() { }

    /** Returns the adapter for a built-in localizer, or a monitor-only adapter for custom code. */
    public static LocalizerAdapter create(BaseLocalizerConstants<?> config) {
        if (config instanceof TankDriveEncoders.Constants) { return new TankAdapter(); }
        if (config instanceof MecanumDriveEncoders.Constants) {
            return new MecanumAdapter((MecanumDriveEncoders.Constants) config);
        }
        if (config instanceof TwoWheel.Constants) { return new TwoWheelAdapter(); }
        if (config instanceof ThreeWheel.Constants) { return new ThreeWheelAdapter(); }
        if (config instanceof Pinpoint.Constants) { return new PinpointAdapter(); }
        if (config instanceof Octoquad.Constants) { return new OctoquadAdapter(); }
        if (config instanceof OTOS.Constants) { return new OtosAdapter(); }
        return new MonitorAdapter(config.getClass().getSimpleName());
    }

    private abstract static class BaseAdapter implements LocalizerAdapter {
        private final String name;
        BaseAdapter(String name) { this.name = name; }
        @Override public String getName() { return name; }
        @Override public boolean supportsDistance(CalibrationAxis axis) { return true; }
        @Override public boolean supportsSpinCalibration() { return false; }
        @Override public CalibrationSnapshot snapshot(BaseLocalizer<?> localizer) {
            return new CalibrationSnapshot(localizer.getPose(), ticks(localizer));
        }
        @Override public CalibrationCandidate fitDirections(BaseLocalizerConstants<?> config,
                                                             List<DirectionTrial> trials) {
            return candidate(copy(config), "No encoder direction settings are required.");
        }
        int[] ticks(BaseLocalizer<?> localizer) { return new int[0]; }
        JSONObject copy(BaseLocalizerConstants<?> config) {
            return copyValues(config);
        }
        double reported(CalibrationAxis axis, CalibrationSnapshot start,
                        CalibrationSnapshot end) {
            Vector fieldDelta = end.pose.getVec().minus(start.pose.getVec());
            Vector robotDelta = fieldDelta.rotate(start.pose.getHeading().times(-1.0));
            return axis == CalibrationAxis.FORWARD
                    ? robotDelta.getX().getIn() : robotDelta.getY().getIn();
        }
        CalibrationCandidate oneScale(BaseLocalizerConstants<?> config, String key,
                                      boolean inverse, CalibrationAxis axis,
                                      CalibrationSnapshot start, CalibrationSnapshot end,
                                      double measured) {
            double estimated = Math.abs(reported(axis, start, end));
            requireDistance(measured, estimated);
            JSONObject values = copy(config);
            double old = values.optDouble(key, 1.0);
            double next = inverse ? old * estimated / measured : old * measured / estimated;
            put(values, key, next);
            return candidate(values, "Reported: " + format(estimated) + " in",
                    "Measured: " + format(measured) + " in",
                    key + ": " + format(old) + " -> " + format(next));
        }
        @Override public CalibrationCandidate fitSpin(BaseLocalizerConstants<?> config,
                                                       List<SpinTrial> trials) {
            throw new UnsupportedOperationException(name + " has no spin-fit geometry");
        }
    }

    private static final class MonitorAdapter extends BaseAdapter {
        MonitorAdapter(String name) { super(name); }
        @Override public boolean supportsDistance(CalibrationAxis axis) { return false; }
        @Override public CalibrationCandidate fitDistance(BaseLocalizerConstants<?> config,
                CalibrationAxis axis, CalibrationSnapshot start, CalibrationSnapshot end,
                double measuredInches) {
            throw new UnsupportedOperationException("Custom localizer is monitor/filter-only");
        }
    }

    private static final class MecanumAdapter extends BaseAdapter {
        private final boolean hasStrafeEncoders;
        MecanumAdapter(MecanumDriveEncoders.Constants config) {
            super("Mecanum drive encoders + IMU");
            hasStrafeEncoders = config.backLeftName != null && config.backRightName != null;
        }
        @Override public boolean supportsDistance(CalibrationAxis axis) {
            return axis == CalibrationAxis.FORWARD || hasStrafeEncoders;
        }
        @Override int[] ticks(BaseLocalizer<?> localizer) {
            return ((MecanumDriveEncoders) localizer).getEncoderTicks();
        }
        @Override public CalibrationCandidate fitDirections(BaseLocalizerConstants<?> raw,
                                                             List<DirectionTrial> trials) {
            MecanumDriveEncoders.Constants config = (MecanumDriveEncoders.Constants) raw;
            DirectionTrial forward = trial(trials, CalibrationAxis.FORWARD);
            JSONObject values = copy(config);
            put(values, "frontLeftReversed", corrected(config.frontLeftReversed,
                    delta(forward, 0, "front-left encoder")));
            put(values, "frontRightReversed", corrected(config.frontRightReversed,
                    delta(forward, 1, "front-right encoder")));
            if (config.backLeftName != null && config.backRightName != null) {
                put(values, "backLeftReversed", corrected(config.backLeftReversed,
                        delta(forward, 2, "back-left encoder")));
                put(values, "backRightReversed", corrected(config.backRightReversed,
                        delta(forward, 3, "back-right encoder")));
            }
            return candidate(values, "Encoder directions ready to save.");
        }
        @Override public CalibrationCandidate fitDistance(BaseLocalizerConstants<?> config,
                CalibrationAxis axis, CalibrationSnapshot start, CalibrationSnapshot end,
                double measured) {
            return oneScale(config, "ticksPerInch", true, axis, start, end, measured);
        }
    }

    private static final class TankAdapter extends BaseAdapter {
        TankAdapter() { super("Tank drive encoders + IMU"); }
        @Override public boolean supportsDistance(CalibrationAxis axis) {
            return axis == CalibrationAxis.FORWARD;
        }
        @Override int[] ticks(BaseLocalizer<?> localizer) {
            return ((TankDriveEncoders) localizer).getEncoderTicks();
        }
        @Override public CalibrationCandidate fitDirections(BaseLocalizerConstants<?> raw,
                                                             List<DirectionTrial> trials) {
            TankDriveEncoders.Constants config = (TankDriveEncoders.Constants) raw;
            DirectionTrial forward = trial(trials, CalibrationAxis.FORWARD);
            JSONObject values = copy(config);
            JSONArray left = values.optJSONArray("left");
            JSONArray right = values.optJSONArray("right");
            int channel = 0;
            channel = setTankDirections(config.leftEncoders, left, forward, channel);
            setTankDirections(config.rightEncoders, right, forward, channel);
            return candidate(values, "Encoder directions ready to save.");
        }
        @Override public CalibrationCandidate fitDistance(BaseLocalizerConstants<?> raw,
                CalibrationAxis axis, CalibrationSnapshot start, CalibrationSnapshot end,
                double measured) {
            TankDriveEncoders.Constants config = (TankDriveEncoders.Constants) raw;
            if (start.rawTicks.length != config.leftEncoders.size() + config.rightEncoders.size()) {
                throw new IllegalStateException("Tank encoder channel count changed during trial");
            }
            JSONObject values = copy(config);
            JSONArray left = values.optJSONArray("left");
            JSONArray right = values.optJSONArray("right");
            List<String> metrics = new ArrayList<>();
            int index = 0;
            index = scaleSide(config.leftEncoders, left, start, end, measured, index, "Left", metrics);
            scaleSide(config.rightEncoders, right, start, end, measured, index, "Right", metrics);
            return new CalibrationCandidate(values, metrics);
        }
        private int scaleSide(List<TankDriveEncoders.Encoder> specs, JSONArray output,
                              CalibrationSnapshot start, CalibrationSnapshot end,
                              double measured, int index, String side, List<String> metrics) {
            double sum = 0.0;
            for (int i = 0; i < specs.size(); i++, index++) {
                TankDriveEncoders.Encoder spec = specs.get(i);
                int delta = end.rawTicks[index] - start.rawTicks[index];
                double distance = delta * (spec.reversed ? -1.0 : 1.0) / spec.ticksPerInch;
                sum += Math.abs(distance);
            }
            double estimated = sum / specs.size();
            requireDistance(measured, estimated);
            double factor = estimated / measured;
            for (int i = 0; i < specs.size(); i++) {
                JSONObject value = output.optJSONObject(i);
                put(value, "ticksPerInch", specs.get(i).ticksPerInch * factor);
            }
            metrics.add(side + " reported: " + format(estimated) + " in");
            metrics.add(side + " scale multiplier: " + format(factor));
            return index;
        }
    }

    private static final class TwoWheelAdapter extends BaseAdapter {
        TwoWheelAdapter() { super("Two-wheel odometry + IMU"); }
        @Override public boolean supportsSpinCalibration() { return true; }
        @Override int[] ticks(BaseLocalizer<?> localizer) { return ((TwoWheel) localizer).getPodTicks(); }
        @Override public CalibrationCandidate fitDirections(BaseLocalizerConstants<?> raw,
                                                             List<DirectionTrial> trials) {
            TwoWheel.Constants config = (TwoWheel.Constants) raw;
            JSONObject values = copy(config);
            put(values, "forwardPodReversed", corrected(config.forwardPodReversed,
                    delta(trial(trials, CalibrationAxis.FORWARD), 0, "forward pod")));
            DirectionTrial strafe = findTrial(trials, CalibrationAxis.STRAFE);
            if (strafe != null) {
                put(values, "strafePodReversed", corrected(config.strafePodReversed,
                        delta(strafe, 1, "strafe pod")));
            }
            return candidate(values, "Pod directions ready to save.");
        }
        @Override public CalibrationCandidate fitDistance(BaseLocalizerConstants<?> config,
                CalibrationAxis axis, CalibrationSnapshot start, CalibrationSnapshot end,
                double measured) {
            return oneScale(config, "ticksPerInch", true, axis, start, end, measured);
        }
        @Override public CalibrationCandidate fitSpin(BaseLocalizerConstants<?> config,
                                                       List<SpinTrial> trials) {
            JSONObject values = copy(config);
            TwoWheel.Constants twoWheel = (TwoWheel.Constants) config;
            double tpi = requirePositive(values.optDouble("ticksPerInch"), "ticksPerInch");
            double[] offsets = rawOffsets(trials, tpi, new int[] {0, 1}, new double[] {
                    twoWheel.forwardPodReversed ? -1 : 1,
                    twoWheel.strafePodReversed ? -1 : 1});
            put(values, "xIn", offsets[0]); put(values, "yIn", offsets[1]);
            return spinCandidate(values, trials, offsets, "Forward offset", "Strafe offset");
        }
    }

    private static final class ThreeWheelAdapter extends BaseAdapter {
        ThreeWheelAdapter() { super("Three-wheel odometry"); }
        @Override public boolean supportsSpinCalibration() { return true; }
        @Override int[] ticks(BaseLocalizer<?> localizer) { return ((ThreeWheel) localizer).getPodTicks(); }
        @Override public CalibrationCandidate fitDirections(BaseLocalizerConstants<?> raw,
                                                             List<DirectionTrial> trials) {
            ThreeWheel.Constants config = (ThreeWheel.Constants) raw;
            DirectionTrial forward = trial(trials, CalibrationAxis.FORWARD);
            JSONObject values = copy(config);
            put(values, "forwardLeftPodReversed", corrected(config.forwardLeftPodReversed,
                    delta(forward, 0, "forward-left pod")));
            put(values, "forwardRightPodReversed", corrected(config.forwardRightPodReversed,
                    delta(forward, 1, "forward-right pod")));
            DirectionTrial strafe = findTrial(trials, CalibrationAxis.STRAFE);
            if (strafe != null) {
                put(values, "strafePodReversed", corrected(config.strafePodReversed,
                        delta(strafe, 2, "strafe pod")));
            }
            return candidate(values, "Pod directions ready to save.");
        }
        @Override public CalibrationCandidate fitDistance(BaseLocalizerConstants<?> config,
                CalibrationAxis axis, CalibrationSnapshot start, CalibrationSnapshot end,
                double measured) {
            return oneScale(config, "ticksPerInch", true, axis, start, end, measured);
        }
        @Override public CalibrationCandidate fitSpin(BaseLocalizerConstants<?> config,
                                                       List<SpinTrial> trials) {
            ThreeWheel.Constants threeWheel = (ThreeWheel.Constants) config;
            JSONObject values = copy(config);
            double tpi = requirePositive(values.optDouble("ticksPerInch"), "ticksPerInch");
            double[] results = new double[2];
            for (SpinTrial trial : trials) {
                requireSpin(trial);
                int left = signedDelta(trial, 0, threeWheel.forwardLeftPodReversed);
                int right = signedDelta(trial, 1, threeWheel.forwardRightPodReversed);
                int strafe = signedDelta(trial, 2, threeWheel.strafePodReversed);
                results[0] += (left - right) / tpi / trial.angleRad;
                results[1] += strafe / tpi / trial.angleRad;
            }
            results[0] /= trials.size(); results[1] /= trials.size();
            put(values, "xIn", results[0]); put(values, "yIn", results[1]);
            return spinCandidate(values, trials, results, "Parallel-pod separation",
                    "Perpendicular offset");
        }
    }

    private abstract static class ComputerAdapter extends BaseAdapter {
        ComputerAdapter(String name) { super(name); }
        abstract double countsPerInch(BaseLocalizerConstants<?> config);
        @Override public boolean supportsSpinCalibration() { return true; }
        @Override public CalibrationCandidate fitSpin(BaseLocalizerConstants<?> config,
                                                       List<SpinTrial> trials) {
            double[] offsets = rawOffsets(trials, countsPerInch(config),
                    new int[] {0, 1}, new double[] {1, 1});
            JSONObject values = copy(config);
            put(values, "xIn", offsets[0]); put(values, "yIn", offsets[1]);
            double reportedAngle = totalReportedAngle(trials);
            double reference = totalReferenceAngle(trials);
            double current = values.optDouble("angularScalar", 1.0);
            if (current == 0.0) { current = 1.0; }
            if (reportedAngle > 0.1) { put(values, "angularScalar", current * reference / reportedAngle); }
            return spinCandidate(values, trials, offsets, "X pod offset", "Y pod offset");
        }
    }

    private static final class PinpointAdapter extends ComputerAdapter {
        PinpointAdapter() { super("goBILDA Pinpoint"); }
        @Override int[] ticks(BaseLocalizer<?> localizer) { return ((Pinpoint) localizer).getPodTicks(); }
        @Override public CalibrationCandidate fitDirections(BaseLocalizerConstants<?> raw,
                                                             List<DirectionTrial> trials) {
            Pinpoint.Constants config = (Pinpoint.Constants) raw;
            JSONObject values = copy(config);
            put(values, "xPodDirection", corrected(config.xPodDirection.name(),
                    delta(trial(trials, CalibrationAxis.FORWARD), 0, "X pod")));
            DirectionTrial strafe = findTrial(trials, CalibrationAxis.STRAFE);
            if (strafe != null) {
                put(values, "yPodDirection", corrected(config.yPodDirection.name(),
                        delta(strafe, 1, "Y pod")));
            }
            return candidate(values, "Pod directions ready to save.");
        }
        @Override double countsPerInch(BaseLocalizerConstants<?> raw) {
            Pinpoint.Constants config = (Pinpoint.Constants) raw;
            if (config.customEncoderResolution.getIn() > 0) {
                return config.customEncoderResolution.getMm() * 25.4;
            }
            return (config.encoderResolution == Pinpoint.GoBildaPods.goBILDA_SWINGARM_POD
                    ? 13.26291192 : 19.89436789) * 25.4;
        }
        @Override public CalibrationCandidate fitDistance(BaseLocalizerConstants<?> config,
                CalibrationAxis axis, CalibrationSnapshot start, CalibrationSnapshot end,
                double measured) {
            Pinpoint.Constants pinpoint = (Pinpoint.Constants) config;
            double storedResolutionIn = pinpoint.customEncoderResolution.getIn() > 0
                    ? pinpoint.customEncoderResolution.getIn()
                    : ((pinpoint.encoderResolution == Pinpoint.GoBildaPods.goBILDA_SWINGARM_POD
                    ? 13.26291192 : 19.89436789) / 25.4);
            return measuredScalar(config, axis, start, end, measured,
                    "customEncoderResolutionIn", true, storedResolutionIn);
        }
    }

    private static final class OctoquadAdapter extends ComputerAdapter {
        OctoquadAdapter() { super("Digital Chicken Labs Octoquad"); }
        @Override int[] ticks(BaseLocalizer<?> localizer) { return ((Octoquad) localizer).getPodTicks(); }
        @Override public CalibrationCandidate fitDirections(BaseLocalizerConstants<?> raw,
                                                             List<DirectionTrial> trials) {
            Octoquad.Constants config = (Octoquad.Constants) raw;
            JSONObject values = copy(config);
            put(values, "xPodDirection", corrected(config.xPodDirection.name(),
                    delta(trial(trials, CalibrationAxis.FORWARD), 0, "X pod")));
            DirectionTrial strafe = findTrial(trials, CalibrationAxis.STRAFE);
            if (strafe != null) {
                put(values, "yPodDirection", corrected(config.yPodDirection.name(),
                        delta(strafe, 1, "Y pod")));
            }
            return candidate(values, "Pod directions ready to save.");
        }
        @Override double countsPerInch(BaseLocalizerConstants<?> raw) {
            return ((Octoquad.Constants) raw).encoderResolution.get(DistUnit.MM) * 25.4;
        }
        @Override public CalibrationCandidate fitDistance(BaseLocalizerConstants<?> config,
                CalibrationAxis axis, CalibrationSnapshot start, CalibrationSnapshot end,
                double measured) {
            return measuredScalar(config, axis, start, end, measured,
                    "encoderResolutionIn", true,
                    requirePositive(copy(config).optDouble("encoderResolutionIn"),
                            "encoderResolutionIn"));
        }
    }

    private static final class OtosAdapter extends BaseAdapter {
        OtosAdapter() { super("SparkFun OTOS"); }
        @Override public boolean supportsSpinCalibration() { return true; }
        @Override public CalibrationCandidate fitDistance(BaseLocalizerConstants<?> config,
                CalibrationAxis axis, CalibrationSnapshot start, CalibrationSnapshot end,
                double measured) {
            return oneScale(config, "linearScalar", false, axis, start, end, measured);
        }
        @Override public CalibrationCandidate fitSpin(BaseLocalizerConstants<?> config,
                                                       List<SpinTrial> trials) {
            JSONObject values = copy(config);
            double reference = 0.0;
            double reported = 0.0;
            for (SpinTrial trial : trials) {
                requireSpin(trial);
                reference += Math.abs(trial.angleRad);
                reported += Math.abs(trial.reportedAngleRad);
            }
            double current = values.optDouble("angularScalar", 1.0);
            if (reported > 0.1) {
                put(values, "angularScalar", current * reference / reported);
            }
            return candidate(values,
                    "Reference rotation: " + format(reference) + " rad",
                    "Reported rotation: " + format(reported) + " rad",
                    "angularScalar: " + format(current) + " -> "
                            + format(values.optDouble("angularScalar", current)),
                    "OTOS offset is preserved; closed full turns cannot identify it.");
        }
    }

    private static int setTankDirections(List<TankDriveEncoders.Encoder> specs, JSONArray output,
                                         DirectionTrial forward, int channel) {
        for (int i = 0; i < specs.size(); i++, channel++) {
            TankDriveEncoders.Encoder spec = specs.get(i);
            JSONObject value = output.optJSONObject(i);
            put(value, "reversed", corrected(spec.reversed,
                    delta(forward, channel, spec.name)));
        }
        return channel;
    }

    private static DirectionTrial trial(List<DirectionTrial> trials, CalibrationAxis axis) {
        DirectionTrial result = findTrial(trials, axis);
        if (result != null) { return result; }
        throw new IllegalArgumentException(axis + " direction trial is missing");
    }

    private static DirectionTrial findTrial(List<DirectionTrial> trials, CalibrationAxis axis) {
        for (DirectionTrial value : trials) {
            if (value.axis == axis) { return value; }
        }
        return null;
    }

    private static int delta(DirectionTrial trial, int channel, String label) {
        if (channel >= trial.start.rawTicks.length || channel >= trial.end.rawTicks.length) {
            throw new IllegalArgumentException(label + " channel is unavailable");
        }
        int value = trial.end.rawTicks[channel] - trial.start.rawTicks[channel];
        if (Math.abs(value) < 5) {
            throw new IllegalArgumentException(label + " did not move enough");
        }
        return value;
    }

    private static int signedDelta(SpinTrial trial, int channel, boolean reversed) {
        int value = trial.end.rawTicks[channel] - trial.start.rawTicks[channel];
        return reversed ? -value : value;
    }

    private static boolean corrected(boolean currentlyReversed, int rawDelta) {
        double directedDelta = rawDelta * (currentlyReversed ? -1.0 : 1.0);
        return directedDelta >= 0.0 ? currentlyReversed : !currentlyReversed;
    }

    private static String corrected(String current, int configuredDelta) {
        if (configuredDelta >= 0) { return current; }
        return "REVERSED".equals(current) ? "FORWARD" : "REVERSED";
    }

    private static CalibrationCandidate measuredScalar(BaseLocalizerConstants<?> config,
            CalibrationAxis axis, CalibrationSnapshot start, CalibrationSnapshot end,
            double measured, String key, boolean inverse, double current) {
        double estimated = Math.abs(reportedDistance(axis, start, end));
        requireDistance(measured, estimated);
        JSONObject values = copyValues(config);
        double next = inverse ? current * estimated / measured : current * measured / estimated;
        put(values, key, next);
        return candidate(values, "Reported: " + format(estimated) + " in",
                "Measured: " + format(measured) + " in", key + ": " + format(next));
    }

    private static JSONObject copyValues(BaseLocalizerConstants<?> config) {
        try { return new JSONObject(config.getCalibrationValues().toString()); }
        catch (Exception e) { throw new IllegalStateException("Could not copy calibration values", e); }
    }

    private static double requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and positive");
        }
        return value;
    }

    private static double reportedDistance(CalibrationAxis axis, CalibrationSnapshot start,
                                           CalibrationSnapshot end) {
        Vector fieldDelta = end.pose.getVec().minus(start.pose.getVec());
        Vector robotDelta = fieldDelta.rotate(start.pose.getHeading().times(-1.0));
        return axis == CalibrationAxis.FORWARD
                ? robotDelta.getX().getIn() : robotDelta.getY().getIn();
    }

    private static double[] rawOffsets(List<SpinTrial> trials, double countsPerInch,
                                       int[] channels, double[] signs) {
        if (!Double.isFinite(countsPerInch) || countsPerInch <= 0) {
            throw new IllegalArgumentException("Encoder resolution must be positive");
        }
        double[] result = new double[channels.length];
        for (SpinTrial trial : trials) {
            requireSpin(trial);
            for (int i = 0; i < channels.length; i++) {
                int channel = channels[i];
                int delta = trial.end.rawTicks[channel] - trial.start.rawTicks[channel];
                result[i] += signs[i] * delta / countsPerInch / trial.angleRad;
            }
        }
        for (int i = 0; i < result.length; i++) { result[i] /= trials.size(); }
        return result;
    }

    private static CalibrationCandidate spinCandidate(JSONObject values, List<SpinTrial> trials,
            double[] offsets, String first, String second) {
        List<String> metrics = new ArrayList<>(Arrays.asList(
                first + ": " + format(offsets[0]) + " in",
                second + ": " + format(offsets[1]) + " in"));
        return new CalibrationCandidate(values, metrics);
    }

    private static double totalReferenceAngle(List<SpinTrial> trials) {
        double value = 0.0;
        for (SpinTrial trial : trials) { value += Math.abs(trial.angleRad); }
        return value;
    }

    private static double totalReportedAngle(List<SpinTrial> trials) {
        double value = 0.0;
        for (SpinTrial trial : trials) { value += Math.abs(trial.reportedAngleRad); }
        return value;
    }

    private static void requireDistance(double measured, double estimated) {
        if (!Double.isFinite(measured) || measured < 2.0) {
            throw new IllegalArgumentException("Measured distance must be at least 2 inches");
        }
        if (!Double.isFinite(estimated) || estimated < 0.5) {
            throw new IllegalArgumentException("Localizer reported insufficient travel");
        }
    }

    private static void requireSpin(SpinTrial trial) {
        if (!Double.isFinite(trial.angleRad) || Math.abs(trial.angleRad) < Math.PI) {
            throw new IllegalArgumentException("Each spin trial needs at least half a revolution");
        }
        if (trial.start.rawTicks.length != trial.end.rawTicks.length) {
            throw new IllegalArgumentException("Raw channel count changed during spin");
        }
    }

    private static void put(JSONObject object, String key, double value) {
        if (!Double.isFinite(value)) { throw new IllegalArgumentException(key + " is not finite"); }
        try { object.put(key, value); }
        catch (Exception e) { throw new IllegalStateException(e); }
    }

    private static void put(JSONObject object, String key, boolean value) {
        try { object.put(key, value); }
        catch (Exception e) { throw new IllegalStateException(e); }
    }

    private static void put(JSONObject object, String key, String value) {
        try { object.put(key, value); }
        catch (Exception e) { throw new IllegalStateException(e); }
    }

    private static CalibrationCandidate candidate(JSONObject values, String... metrics) {
        return new CalibrationCandidate(values, Arrays.asList(metrics));
    }

    private static String format(double value) {
        return String.format(java.util.Locale.US, "%.5f", value);
    }
}
