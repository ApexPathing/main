package commands;

public interface Command {
    default void initialize() {

    }
    default void execute() {

    }
    default void end(boolean interrupted) {

    }
    default boolean isFinished() {
        return false;
    }
    default Command whenFinished(Runnable toRun) {
        return new SequentialCommandGroup(this, new InstantCommand(toRun));//TODO
    }
    default Command beforeStarting(Runnable toRun) {
        return new SequentialCommandGroup (new InstantCommand(toRun), this);
    }
    default Command andThen(Command... next) {
        SequentialCommandGroup group = new SequentialCommandGroup(
                this
        );
        group.addCommands(next);
        return group;
    }
    default Command alongWith(Command... parallel) {
        ParallelCommandGroup group = new ParallelCommandGroup(this);
        group.addCommands(parallel);
        return group;
    }
    default void schedule(boolean interruptible) {
        CommandScheduler.getInstance().schedule(interruptible, this);
    }
    default void schedule() {
        schedule(true);
    }
    default void cancel() {
        CommandScheduler.getInstance().cancel(this);
    }

    default boolean isScheduled() {
        return CommandScheduler.getInstance().isScheduled(this);
    }

    default boolean runsWhenDisabled() {
        return false;
    }

    default String getName() {
        return this.getClass().getSimpleName();
    }

}
