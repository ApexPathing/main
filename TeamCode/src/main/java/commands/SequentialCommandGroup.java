package commands;

import java.util.ArrayList;
import java.util.List;
/**
 * @author Sohum Arora 22985 Paraducks
 */

public class SequentialCommandGroup extends CommandGroupBase {
    private final List<Command> commandsList = new ArrayList<>();
    private int currentCommandIndex = -1;
    private boolean runWhenDisabled = true;

    public SequentialCommandGroup(Command... commands){
        addCommands(commands);
    }
    public void addCommands(Command...commands) {
        requireUngrouped(commands);
        if (currentCommandIndex != -1) {
            throw new IllegalStateException(
              "Commands can't be added to a CommandGroup while the group is running"
            );
        }
        for (Command command : commands) {
            commandsList.add(command);
            runWhenDisabled &= command.runsWhenDisabled();
        }
    }
    @Override
    public void initialize() {
        currentCommandIndex = 0;

        if (!commandsList.isEmpty()) {
            commandsList.get(0).initialize();
        }
    }
    @Override
    public void execute() {
        if (commandsList.isEmpty()) {
            return;
        }

        Command currentCommand = commandsList.get(currentCommandIndex);
        currentCommand.execute();

        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            currentCommandIndex ++;
            if (currentCommandIndex < commandsList.size()) {
                commandsList.get(currentCommandIndex).initialize();
            }
        }
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted && !commandsList.isEmpty()) {
            commandsList.get(currentCommandIndex).end(true);
        }
        currentCommandIndex = -1;
    }

    @Override
    public boolean isFinished() {
        return currentCommandIndex == commandsList.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return runWhenDisabled;
    }

}
