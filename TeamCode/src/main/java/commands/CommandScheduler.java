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
 * @author Atharv G - 13085 Bionic Dutch
 *
 * Schedules and runs commands and command groups.
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

    public CommandScheduler() {}

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

    /**
     * Adds a command to the scheduler to check for every loop.
     * @param interruptible Dictates if the command can be safely interrupted
     * @param command       Action that will run when run() is called
     */
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

    /**
     * Adds commands to the scheduler to check for every loop.
     * @param interruptible Dictates if the command can be safely interrupted
     * @param commands       Actions that will run when run() is called
     */
    public void schedule(boolean interruptible, Command... commands) {
        for (Command command : commands) {
            schedule(interruptible, command);
        }
    }

    /**
     * Adds commands to the scheduler to check for every loop.
     * @param commands       Actions that will run when run() is called
     */
    public void schedule(Command... commands) {
        schedule(true, commands);
    }

    /**
     * Updates the scheduler, running all commands with a criteria that returns true.
     */
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

    /**
     * Remove commands from schedule queue.
     * @param commands  Commands to cancel
     */
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

    /**
     * Removes all commands from queue.
     */
    public void cancelAll() {
        for (Command command : new ArrayList<>(scheduledCommands.keySet())) {
            cancel(command);
        }
    }

    /**
     * Checks if commands are queued to run.
     * @param commands  Commands to check
     * @return  A set of booleans indicating if the respective command is scheduled or not.
     */
    public boolean isScheduled(Command... commands) {
        return scheduledCommands.keySet().containsAll(Arrays.asList(commands));
    }

    /**
     * Completely remove commands and other data from scheduler, regardless of if a command is running or not.
     */
    public synchronized void reset() {
        instance = null;
    }

    /**
     * Pauses function of the scheduler run() method, retaining command data.
     */
    public void disable() { disabled = true; }

    /**
     * Resumes function of the scheduler run() method.
     */
    public void enable() { disabled = false; }

    /**
     * Adds an initialization instruction to a registered Command.
     * @param action    The action to run when the command is initialized
     */
    public void onCommandInitialize(Consumer<Command> action) { initActions.add(action); }

    /**
     * Adds an execution instruction to a registered Command.
     * @param action    The action to run when the command is executed
     */
    public void onCommandExecute(Consumer<Command> action) { executeActions.add(action); }

    /**
     * Adds an interruption instruction to a registered Command.
     * @param action    The action to run when the command is interrupted
     */
    public void onCommandInterrupt(Consumer<Command> action) { interruptActions.add(action); }

    /**
     * Adds a finish instruction to a registered Command.
     * @param action    Action to run when the command is finished
     */
    public void onCommandFinish(Consumer<Command> action) { finishActions.add(action); }
}