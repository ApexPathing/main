package commands;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Set;
import java.util.WeakHashMap;

public abstract class CommandGroupBase implements Command {

    private static final Set<Command> groupedCommands =
            Collections.newSetFromMap(new WeakHashMap<>());

    static void registerGroupedCommands(Command... commands) {
        groupedCommands.addAll(Arrays.asList(commands));
    }

    public static void clearGroupedCommands() {
        groupedCommands.clear();
    }

    public static void clearGroupedCommand(Command command) {
        groupedCommands.remove(command);
    }

    public static void requireUngrouped(Command... commands) {
        requireUngrouped(Arrays.asList(commands));
    }

    public static void requireUngrouped(Collection<Command> commands) {
        if (!Collections.disjoint(commands, getGroupedCommands())) {
            throw new IllegalArgumentException("Commands cannot be added to more than one CommandGroup");
        }
    }

    static Set<Command> getGroupedCommands() {
        return groupedCommands;
    }

    public abstract void addCommands(Command... commands);
}