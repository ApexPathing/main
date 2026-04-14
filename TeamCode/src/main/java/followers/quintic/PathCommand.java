package followers.quintic;

//import com.arcrobotics.ftclib.command.CommandBase; //todo


/**
 * FTCLib command that follows a {@link QuinticPath} using a {@link QuinticFollower}.
 * Finishes when the follower is no longer busy.
 * @author Sohum Arora 22985 Paraducks
 */
public class PathCommand /*extends CommandBase todo must add abstract command base*/{

    private final QuinticFollower follower;
    private final QuinticPath path;

    /**
     * Constructor for {@link PathCommand}
     * @param follower the {@link QuinticFollower} to use for path following
     * @param path the {@link QuinticPath} to follow
     */
    public PathCommand(QuinticFollower follower, QuinticPath path) {
        this.follower = follower;
        this.path = path;
    }

//    @Override todo
    public void initialize() {
        follower.followPath(path);
    }

//    @Override todo
    public void execute() {
        follower.update();
    }

//    @Override todo
    public boolean isFinished() {
        return !follower.isBusy();
    }

//    @Override todo
    public void end(boolean interrupted) {
        follower.stop();
    }
}