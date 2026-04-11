package commands;

public class InstantCommand implements Command{
    private final Runnable toRun;

    public InstantCommand(Runnable toRun) {
        this.toRun = toRun;
    }
    public InstantCommand() {
        toRun = ()->{};
    }
    @Override
    public final boolean isFinished(){
        return true;
    }
    @Override
    public void initialize() {
        toRun.run();
    }

}
