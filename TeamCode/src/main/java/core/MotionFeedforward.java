package core;

import com.qualcomm.robotcore.util.Range;

/** Pure feedforward calculations shared by holonomic and tank path execution. */
final class MotionFeedforward {
    private MotionFeedforward() { }

    static double motionSign(double targetVelocity, double targetAcceleration) {
        if (Math.abs(targetVelocity) > 1e-6) {
            return Math.signum(targetVelocity);
        }
        if (Math.abs(targetAcceleration) > 1e-6) {
            return Math.signum(targetAcceleration);
        }
        return 0.0;
    }

    static double timeScale(double targetTangentialVelocity, double measuredTangentialVelocity) {
        if (!Double.isFinite(targetTangentialVelocity)
                || !Double.isFinite(measuredTangentialVelocity)
                || targetTangentialVelocity <= 1e-6) {
            return 0.0;
        }
        return Range.clip(measuredTangentialVelocity / targetTangentialVelocity, 0.0, 1.0);
    }

    static double scaleBrakingAcceleration(double targetVelocity, double targetAcceleration,
                                           double measuredVelocity) {
        if (targetVelocity * targetAcceleration >= 0.0) {
            return targetAcceleration;
        }
        double scale = timeScale(Math.abs(targetVelocity),
                measuredVelocity * Math.signum(targetVelocity));
        return targetAcceleration * scale * scale;
    }

    static double calculate(double targetVelocity, double targetAcceleration,
                            double kV, double kA, double kS) {
        double motionSign = motionSign(targetVelocity, targetAcceleration);
        if (motionSign == 0.0) {
            return 0.0;
        }
        return kV * targetVelocity + kA * targetAcceleration + motionSign * kS;
    }
}
