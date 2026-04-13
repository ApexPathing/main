package commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * @author Sohum Arora 22985 Paraducks
 */
public final class CommandScheduler {

    private static CommandScheduler instance;

    public static synchronized CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    private final Map<Command, CommandState> scheduledCommands = new LinkedHashMap<>();
    private final Collection<Runnable> buttons = new LinkedHashSet<>();
    private boolean disabled;
    private boolean inRunLoop;

    private final List<Consumer<Command>> initActions = new ArrayList<>();
    private final List<Consumer<Command>> executeActions = new ArrayList<>();
    private final List<Consumer<Command>> interruptActions = new ArrayList<>();
    private final List<Consumer<Command>> finishActions = new ArrayList<>();

    private final Map<Command, Boolean> toSchedule = new LinkedHashMap<>();
    private final List<Command> toCancel = new ArrayList<>();

    CommandScheduler() {}

    public void addButton(Runnable button) {
        buttons.add(button);
    }

    public void clearButtons() {
        buttons.clear();
    }

    private void initCommand(Command command, boolean interruptible) {
        command.initialize();
        scheduledCommands.put(command, new CommandState(interruptible));
        for (Consumer<Command> action : initActions) {
            action.accept(command);
        }
    }

    private void schedule(boolean interruptible, Command command) {
        if (inRunLoop) {
            toSchedule.put(command, interruptible);
            return;
        }

        if (CommandGroupBase.getGroupedCommands().contains(command)) {
            throw new IllegalArgumentException("A command that is part of a command group cannot be independently scheduled");
        }

        if (disabled || scheduledCommands.containsKey(command)) {
            return;
        }

        initCommand(command, interruptible);
    }

    public void schedule(boolean interruptible, Command... commands) {
        for (Command command : commands) {
            schedule(interruptible, command);
        }
    }

    public void schedule(Command... commands) {
        schedule(true, commands);
    }

    public void run() {
        if (disabled) return;

        for (Runnable button : buttons) {
            button.run();
        }

        inRunLoop = true;

        for (Iterator<Command> iterator = scheduledCommands.keySet().iterator(); iterator.hasNext();) {
            Command command = iterator.next();

            command.execute();
            for (Consumer<Command> action : executeActions) {
                action.accept(command);
            }

            if (command.isFinished()) {
                command.end(false);
                for (Consumer<Command> action : finishActions) {
                    action.accept(command);
                }
                iterator.remove();
            }
        }

        inRunLoop = false;

        for (Map.Entry<Command, Boolean> entry : toSchedule.entrySet()) {
            schedule(entry.getValue(), entry.getKey());
        }

        for (Command command : toCancel) {
            cancel(command);
        }

        toSchedule.clear();
        toCancel.clear();
    }

    public void cancel(Command... commands) {
        if (inRunLoop) {
            toCancel.addAll(Arrays.asList(commands));
            return;
        }

        for (Command command : commands) {
            if (!scheduledCommands.containsKey(command)) continue;

            command.end(true);
            for (Consumer<Command> action : interruptActions) {
                action.accept(command);
            }
            scheduledCommands.remove(command);
        }
    }

    public void cancelAll() {
        for (Command command : new ArrayList<>(scheduledCommands.keySet())) {
            cancel(command);
        }
    }

    public boolean isScheduled(Command... commands) {
        return scheduledCommands.keySet().containsAll(Arrays.asList(commands));
    }

    public synchronized void reset() {
        instance = null;
    }

    public void disable() { disabled = true; }
    public void enable() { disabled = false; }

    public void onCommandInitialize(Consumer<Command> action) { initActions.add(action); }
    public void onCommandExecute(Consumer<Command> action) { executeActions.add(action); }
    public void onCommandInterrupt(Consumer<Command> action) { interruptActions.add(action); }
    public void onCommandFinish(Consumer<Command> action) { finishActions.add(action); }
}