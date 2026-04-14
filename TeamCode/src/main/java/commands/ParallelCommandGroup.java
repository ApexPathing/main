package commands;

import java.util.HashMap;
import java.util.Map;
/**
 * @author Sohum Arora 22985 Paraducks
 */

public class ParallelCommandGroup extends CommandGroupBase{
    private final Map<Command, Boolean> commandsList = new HashMap<>();
    private boolean runWhenDisabled = true;

    public ParallelCommandGroup(Command... commands) {
        addCommands(commands);
    }

    @Override
    public void addCommands(Command... commands) {
        requireUngrouped(commands);
        if (commandsList.containsValue(true)){
            throw new IllegalStateException(
                    "Commands cannot be added to a CommandGroup while the group is running");
        }

        registerGroupedCommands(commands);

        for (Command command : commands) {
            commandsList.put(command, false);
            runWhenDisabled &= command.runsWhenDisabled();
        }
    }

    @Override
    public void initialize() {
        for (Map.Entry<Command, Boolean> commandRunning : commandsList.entrySet()) {
            commandRunning.getKey().initialize();
            commandRunning.setValue(true);
        }
    }

    @Override
    public void execute() {
        for (Map.Entry<Command, Boolean> commandRunning : commandsList.entrySet()) {
            if (!commandRunning.getValue()) {
                continue;
            }
            commandRunning.getKey().execute();
            if (commandRunning.getKey().isFinished()) {
                commandRunning.getKey().end(false);
                commandRunning.setValue(false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            for (Map.Entry<Command, Boolean> commandRunning : commandsList.entrySet()) {
                if (commandRunning.getValue()) {
                    commandRunning.getKey().end(true);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !commandsList.values().contains(true);
    }
    @Override
    public boolean runsWhenDisabled() {
        return runWhenDisabled;
    }
}
