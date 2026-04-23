package followers.quintic;

import com.qualcomm.robotcore.util.ElapsedTime;

import drivetrains.Drivetrain;
import followers.Follower;
import localizers.Localizer;
import util.Angle;
import util.Pose;


/**
 * Pure-pursuit follower for QuinticPath splines.
 * Provides C2-continuous curved path following using quintic Hermite segments.
 * Usage:
 *   QuinticPath path = new QuinticPath.Builder()
 *       .addPoint(0,  0,  0)
 *       .addPoint(24, 24, 90)
 *       .addPoint(48, 0,  0)
 *       .build();
 *   follower.followPath(path);
 *   while (follower.isBusy()) {
 *       localizer.update();
 *       follower.update();
 *   }
 * @author Sohum Arora 22985 Paraducks
 */
public class QuinticFollower extends Follower {

    // ── Tuning constants ───────────────────────────────────────────────────────

    /** Lookahead distance in inches */
    public double LookaheadDist = 8.0;

    /** Max translational speed [0,1] */
    public double MaxSpeed = 0.8;

    /** Min speed near end of path */
    public double MinSpeed = 0.15;

    /** Distance from end at which robot starts slowing down (inches) */
    public double SlowDownDist = 12.0;

    /** Stop when within this distance of end (inches) */
    public double EndThreshold = 1.5;

    /** Heading P gain */
    public double HeadingP = 1.2;

    /** Heading D gain */
    public double HeadingD = 0.08;

    // ── Internal state ─────────────────────────────────────────────────────────

    private QuinticPath Path;
    private double PrevHeadingError = 0;
    private final ElapsedTime Timer = new ElapsedTime();
    private double PrevTime = 0;

    public QuinticFollower(Drivetrain drivetrain, Localizer localizer) {
        super(drivetrain, localizer);
    }

    /**
     * Start following a quintic path.
     * Sets isBusy = true via setTargetPose().
     */
    public void followPath(QuinticPath path) {
        this.Path = path;
        PrevHeadingError = 0;
        PrevTime = 0;
        Timer.reset();
        setTargetPose(path.getPoseAtDistance(path.getLength())); // sets isBusy = true
    }

    // ── Follower implementation ────────────────────────────────────────────────

    @Override
    public void update() {
        if (!isBusy || Path == null) return;

        Pose current = localizer.getPose();
        double x = current.getX();
        double y = current.getY();
        double heading = current.getHeading();

        // Check if we've reached the end
        if (Path.isNearEnd(x, y, EndThreshold)) {
            stop(); // inherited — clears isBusy and targetPose
            return;
        }

        // Lookahead target pose on path
        Pose target = Path.getLookaheadPose(x, y, LookaheadDist);

        // Field-frame vector to target
        double fieldDx = target.getX() - x;
        double fieldDy = target.getY() - y;

        // Rotate into robot frame (field-centric → robot-centric)
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double robotDx = fieldDx * cos - fieldDy * sin;
        double robotDy = fieldDx * sin + fieldDy * cos;

        // Normalize translation
        double norm = Math.hypot(robotDx, robotDy);
        if (norm > 1e-6) { robotDx /= norm; robotDy /= norm; }

        // Speed scaling: slow down as we approach the end
        Pose endPose = Path.getPoseAtDistance(Path.getLength());
        double distToEnd = Math.hypot(x - endPose.getX(), y - endPose.getY());
        double speed = (distToEnd < SlowDownDist)
                ? MinSpeed + (MaxSpeed - MinSpeed) * (distToEnd / SlowDownDist)
                : MaxSpeed;

        // Heading PD control toward target heading
        double dt = Math.max(Timer.seconds() - PrevTime, 0.001);
        PrevTime = Timer.seconds();

        double headingError = Angle.normalize(Math.toRadians(target.getHeading()) - Math.toRadians(heading));
        double headingDeriv = (headingError - PrevHeadingError) / dt;
        PrevHeadingError = headingError;

        double turn = HeadingP * headingError + HeadingD * headingDeriv;
        turn = Math.max(-1, Math.min(1, turn));

        // robotDy = forward, robotDx = strafe (standard FTC mecanum convention)
        drive(robotDx, robotDy, turn, heading);
    }

    /**
     * Progress along the current path as fraction [0, 1].
     */
    public double getProgress() {
        if (Path == null) return 0;
        Pose current = localizer.getPose();
        return Path.getProgress(current.getX(), current.getY());
    }
}