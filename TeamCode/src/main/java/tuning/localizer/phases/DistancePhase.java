package tuning.localizer.phases;

import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.LocalizerAdapter;
import tuning.localizer.TuningPhase;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

/** Calibrates translation scale using equal powered trials in both directions. */
public final class DistancePhase extends TuningPhase {
    private static final double POWER = 0.50;
    private static final long DRIVE_NANOS = 1_000_000_000L;
    private static final long SETTLE_NANOS = 400_000_000L;

    private final CalibrationAxis axis;
    private final List<CalibrationCandidate> trials = new ArrayList<>();
    private CalibrationSnapshot start;
    private CalibrationSnapshot pendingEnd;
    private long stageStarted;
    private int direction;
    private Stage stage;
    private double measuredInches;
    private double increment;
    private CalibrationCandidate candidate;

    private enum Stage { DRIVE, SETTLE, ENTER_DISTANCE }

    public DistancePhase(LocalizationTunerContext context, CalibrationAxis axis) {
        super(context);
        this.axis = axis;
    }

    @Override protected String getName() {
        return (axis == CalibrationAxis.FORWARD ? "Forward" : "Strafe") + " distance";
    }

    @Override protected void reset() {
        trials.clear();
        candidate = null;
        direction = 1;
        increment = 0.25;
    }

    @Override protected void showPrepare() {
        context.getTelemetry().addLine("Mark the robot's start position on the floor.");
        context.getTelemetry().addLine("The robot will drive at 50% power for one second twice.");
        context.getTelemetry().addLine("Measure actual travel after each direction; signs are automatic.");
        context.getTelemetry().addLine("A: begin");
    }

    @Override protected void beginRecording() {
        start = context.getAdapter().snapshot(context.getLocalizer());
        stageStarted = System.nanoTime();
        stage = Stage.DRIVE;
    }

    @Override protected boolean record() {
        if (stage == Stage.DRIVE) {
            double forward = axis == CalibrationAxis.FORWARD ? POWER * direction : 0.0;
            double strafe = axis == CalibrationAxis.STRAFE ? POWER * direction : 0.0;
            context.getDrivetrain().moveWithVectors(forward, strafe, 0.0);
            context.getTelemetry().addData("Trial direction", direction > 0 ? "positive" : "negative");
            if (System.nanoTime() - stageStarted >= DRIVE_NANOS) {
                context.stop();
                stage = Stage.SETTLE;
                stageStarted = System.nanoTime();
            }
            return false;
        }
        context.stop();
        if (stage == Stage.SETTLE) {
            context.getTelemetry().addLine("Settling...");
            if (System.nanoTime() - stageStarted >= SETTLE_NANOS) {
                pendingEnd = context.getAdapter().snapshot(context.getLocalizer());
                double reported = reportedDistance(start, pendingEnd);
                measuredInches = Math.max(0.25, Math.abs(reported));
                stage = Stage.ENTER_DISTANCE;
            }
            return false;
        }

        editMeasuredDistance();
        context.getTelemetry().addData("Measured distance", context.formatNumber(measuredInches) + " in");
        context.getTelemetry().addData("Increment", context.formatNumber(increment) + " in");
        context.getTelemetry().addLine("Dpad Up/Down: edit   Left/Right: change increment   A: confirm");
        if (opMode.gamepad1.aWasPressed()) {
            CalibrationSnapshot end = pendingEnd;
            trials.add(context.getAdapter().fitDistance(
                    context.getConfig(), axis, start, end, measuredInches));
            if (direction > 0) {
                direction = -1;
                start = end;
                stage = Stage.DRIVE;
                stageStarted = System.nanoTime();
            } else {
                candidate = average(trials);
                return true;
            }
        }
        return false;
    }

    @Override protected void showReview() {
        for (String metric : candidate.getMetrics()) { context.getTelemetry().addLine(metric); }
    }

    @Override protected boolean accept() { return context.acceptGeometry(axis.name(), candidate); }

    private void editMeasuredDistance() {
        if (opMode.gamepad1.dpadLeftWasPressed()) { increment = Math.max(0.01, increment / 10.0); }
        if (opMode.gamepad1.dpadRightWasPressed()) { increment = Math.min(10.0, increment * 10.0); }
        if (opMode.gamepad1.dpadUpWasPressed()) { measuredInches += increment; }
        if (opMode.gamepad1.dpadDownWasPressed()) {
            measuredInches = Math.max(0.01, measuredInches - increment);
        }
    }

    private double reportedDistance(CalibrationSnapshot from, CalibrationSnapshot to) {
        geometry.Vector delta = to.pose.getVec().minus(from.pose.getVec())
                .rotate(from.pose.getHeading().times(-1.0));
        return axis == CalibrationAxis.FORWARD
                ? delta.getX().getIn() : delta.getY().getIn();
    }

    private static CalibrationCandidate average(List<CalibrationCandidate> values) {
        if (values.size() != 2) { throw new IllegalStateException("Two distance trials are required"); }
        JSONObject averaged = averageObject(values.get(0).getValues(), values.get(1).getValues());
        List<String> metrics = new ArrayList<>();
        metrics.add("Both powered directions were measured.");
        metrics.addAll(values.get(0).getMetrics());
        metrics.addAll(values.get(1).getMetrics());
        return new CalibrationCandidate(averaged, metrics);
    }

    private static JSONObject averageObject(JSONObject first, JSONObject second) {
        JSONObject output = new JSONObject();
        JSONArray names = first.names();
        if (names == null) { return output; }
        try {
            for (int i = 0; i < names.length(); i++) {
                String key = names.getString(i);
                Object a = first.get(key), b = second.opt(key);
                if (a instanceof Number && b instanceof Number) {
                    output.put(key, (((Number) a).doubleValue() + ((Number) b).doubleValue()) / 2.0);
                } else if (a instanceof JSONObject && b instanceof JSONObject) {
                    output.put(key, averageObject((JSONObject) a, (JSONObject) b));
                } else if (a instanceof JSONArray && b instanceof JSONArray) {
                    output.put(key, averageArray((JSONArray) a, (JSONArray) b));
                } else { output.put(key, a); }
            }
            return output;
        } catch (Exception e) { throw new IllegalStateException("Could not combine distance trials", e); }
    }

    private static JSONArray averageArray(JSONArray first, JSONArray second) throws Exception {
        JSONArray output = new JSONArray();
        for (int i = 0; i < first.length(); i++) {
            Object a = first.get(i), b = i < second.length() ? second.get(i) : null;
            output.put(a instanceof JSONObject && b instanceof JSONObject
                    ? averageObject((JSONObject) a, (JSONObject) b) : a);
        }
        return output;
    }
}
