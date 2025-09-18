package org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

/**
 * A command group that runs multiple commands in parallel.
 */
public class ParallelCommandGroup extends Command {

    /** All commands that belong to this group (permanent structure). */
    private final Set<Command> allCommands = Collections.synchronizedSet(new HashSet<>());

    /** Commands that are currently active (reset on initialize). */
    private final Set<Command> activeCommands = Collections.synchronizedSet(new HashSet<>());

    public ParallelCommandGroup(Set<Command> initialCommands) {
        add(initialCommands);
    }

    public ParallelCommandGroup(Command... initialCommands) {
        add(initialCommands);
    }

    /** Add new commands to the group (varargs). */
    public void add(Command... commands) {
        add(Set.of(commands));
    }

    /** Add new commands to the group (Set). */
    public void add(Set<Command> commands) {
        allCommands.addAll(commands);
    }

    /** Remove commands permanently from the group (varargs). */
    public void remove(Command... commands) {
        remove(Set.of(commands));
    }

    /** Remove commands permanently from the group (Set). */
    public void remove(Set<Command> commands) {
        for (Command command : commands) {
            allCommands.remove(command);
            if (activeCommands.contains(command)) {
                command.end(true); // cleanup if it was active
                activeCommands.remove(command);
            }
        }
    }

    @Override
    public void initialize() {
        // Reset active commands from all commands
        activeCommands.clear();
        activeCommands.addAll(allCommands);

        // Initialize all of them
        for (Command command : activeCommands) {
            command.initialize();
        }
    }

    @Override
    public void execute() {
        // Use iterator to allow removals
        Iterator<Command> iterator = activeCommands.iterator();

        while (iterator.hasNext()) {
            Command command = iterator.next();

            // Check if finished BEFORE executing
            if (command.isFinished()) {
                command.end(false);
                iterator.remove();
                continue;
            }

            // Otherwise run normally
            command.execute();

        }
    }

    @Override
    public boolean isFinished() {
        return activeCommands.isEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup any still-active commands
        for (Command command : activeCommands) {
            command.end(true);
        }
        activeCommands.clear();
    }
}
