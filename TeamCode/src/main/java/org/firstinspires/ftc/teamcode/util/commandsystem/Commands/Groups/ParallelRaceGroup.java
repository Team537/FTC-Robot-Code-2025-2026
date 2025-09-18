package org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * A command group that runs multiple commands in parallel,
 * but finishes as soon as any one of the commands finishes.
 */
public class ParallelRaceGroup extends Command {

    /** All commands that belong to this group (permanent structure). */
    private final Set<Command> allCommands = Collections.synchronizedSet(new LinkedHashSet<>());

    /** Commands that are currently active (reset on initialize). */
    private final Set<Command> activeCommands = Collections.synchronizedSet(new LinkedHashSet<>());

    public ParallelRaceGroup(Set<Command> initialCommands) {
        add(initialCommands);
    }

    public ParallelRaceGroup(Command... initialCommands) {
        add(initialCommands);
    }

    /** Add commands (varargs). */
    public void add(Command... commands) {
        Collections.addAll(allCommands, commands);
    }

    /** Add commands (Set). */
    public void add(Set<Command> commands) {
        allCommands.addAll(commands);
    }

    /** Remove commands permanently (varargs). */
    public void remove(Command... commands) {
        remove(Set.of(commands));
    }

    /** Remove commands permanently (Set). */
    public void remove(Set<Command> commands) {
        for (Command command : commands) {
            allCommands.remove(command);
            if (activeCommands.contains(command)) {
                command.end(true); // interrupt if active
                activeCommands.remove(command);
            }
        }
    }

    @Override
    public void initialize() {
        activeCommands.clear();
        activeCommands.addAll(allCommands);

        // Initialize all active commands
        for (Command command : activeCommands) {
            command.initialize();
        }
    }

    @Override
    public void execute() {
        Iterator<Command> iterator = activeCommands.iterator();
        boolean anyFinished = false;

        while (iterator.hasNext()) {
            Command command = iterator.next();

            // Check if finished BEFORE executing
            if (command.isFinished()) {
                anyFinished = true;
                command.end(false);
                iterator.remove();
                continue;
            }

            // Otherwise, run normally
            command.execute();

            // Check if finished DURING execution
            if (command.isFinished()) {
                anyFinished = true;
                command.end(false);
                iterator.remove();
            }
        }

        // If any command finished, the race group ends immediately
        if (anyFinished && !activeCommands.isEmpty()) {
            // Interrupt remaining commands
            for (Command remaining : activeCommands) {
                remaining.end(true);
            }
            activeCommands.clear();
        }
    }

    @Override
    public boolean isFinished() {
        // Race group finishes when all active commands are done
        return activeCommands.isEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup remaining active commands
        for (Command command : activeCommands) {
            command.end(true);
        }
        activeCommands.clear();
    }
}
