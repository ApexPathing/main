package tuning.localizer.phases;

import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.LocalizerAdapter;
import tuning.localizer.TuningPhase;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/**
 * Accumulates rotation about the robot's observed spin axis from relative IMU quaternions.
 * The axis is learned from motion, so hub mounting orientation is not required.
 */
public final class RelativeImuAccumulator {
    private Quaternion previous;
    private double axisX;
    private double axisY;
    private double axisZ;
    private boolean axisKnown;
    private double angle;
    private double offAxisSquared;
    private int samples;

    /** Discards the learned axis before beginning a new independent calibration run. */
    public void clearAxis() {
        previous = null;
        axisKnown = false;
        axisX = axisY = axisZ = 0.0;
        angle = offAxisSquared = 0.0;
        samples = 0;
    }

    /** Clears the angle while retaining the learned physical spin axis. */
    public void resetMeasurement(Quaternion initial) {
        previous = normalized(initial);
        angle = 0.0;
        offAxisSquared = 0.0;
        samples = 0;
    }

    /** Adds a quaternion sample and returns total signed rotation in radians. */
    public double update(Quaternion current) {
        Quaternion next = normalized(current);
        if (previous == null) { resetMeasurement(next); return angle; }
        double w = previous.w * next.w + previous.x * next.x
                + previous.y * next.y + previous.z * next.z;
        double x = previous.w * next.x - previous.x * next.w
                - previous.y * next.z + previous.z * next.y;
        double y = previous.w * next.y + previous.x * next.z
                - previous.y * next.w - previous.z * next.x;
        double z = previous.w * next.z - previous.x * next.y
                + previous.y * next.x - previous.z * next.w;
        if (w < 0.0) { w = -w; x = -x; y = -y; z = -z; }
        double vectorLength = Math.sqrt(x * x + y * y + z * z);
        if (vectorLength > 1e-8) {
            double step = 2.0 * Math.atan2(vectorLength, Math.max(0.0, w));
            double ux = x / vectorLength, uy = y / vectorLength, uz = z / vectorLength;
            if (!axisKnown && step > 1e-4) {
                axisX = ux; axisY = uy; axisZ = uz; axisKnown = true;
            }
            if (axisKnown) {
                double projection = ux * axisX + uy * axisY + uz * axisZ;
                angle += step * projection;
                offAxisSquared += (1.0 - projection * projection) * step * step;
                samples++;
            }
        }
        previous = next;
        return angle;
    }

    public double getAngleRad() { return angle; }
    public boolean hasSpinAxis() { return axisKnown; }
    /** Returns the learned X component of the robot's positive spin axis. */
    public double getSpinAxisX() { return axisKnown ? axisX : 0.0; }
    /** Returns the learned Y component of the robot's positive spin axis. */
    public double getSpinAxisY() { return axisKnown ? axisY : 0.0; }
    /** Returns the learned Z component of the robot's positive spin axis. */
    public double getSpinAxisZ() { return axisKnown ? axisZ : 0.0; }
    public double getRmsOffAxisRad() {
        return samples == 0 ? 0.0 : Math.sqrt(offAxisSquared / samples);
    }

    private static Quaternion normalized(Quaternion value) {
        double norm = Math.sqrt(value.w * value.w + value.x * value.x
                + value.y * value.y + value.z * value.z);
        if (!Double.isFinite(norm) || norm < 1e-12) {
            throw new IllegalArgumentException("IMU returned an invalid quaternion");
        }
        return new Quaternion((float) (value.w / norm), (float) (value.x / norm),
                (float) (value.y / norm), (float) (value.z / norm), value.acquisitionTime);
    }
}
