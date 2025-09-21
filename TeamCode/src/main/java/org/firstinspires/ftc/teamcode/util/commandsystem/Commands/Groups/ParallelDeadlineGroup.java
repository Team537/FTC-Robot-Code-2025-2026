package org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * A command group that runs multiple commands in parallel,
 * but finishes when the specified "deadline" command finishes.
 */
public class ParallelDeadlineGroup extends Command {

    /** The command that determines when the group finishes. */
    private final Command deadline;

    /** All other commands that belong to this group. */
    private final Set<Command> allCommands = Collections.synchronizedSet(new LinkedHashSet<>());

    /** Commands currently active (reset on initialize). */
    private final Set<Command> activeCommands = Collections.synchronizedSet(new LinkedHashSet<>());

    public ParallelDeadlineGroup(Command deadline, Set<Command> otherCommands) {
        this.deadline = deadline;
        add(otherCommands);
    }

    public ParallelDeadlineGroup(Command deadline, Command... otherCommands) {
        this.deadline = deadline;
        add(otherCommands);
    }

    /** Add commands (varargs). */
    public void add(Command... commands) {
        add(Set.of(commands));
    }

    /** Add commands (Set). */
    public void add(Set<Command> commands) {
        allCommands.addAll(commands);

        for (Command command : commands) {
            addRequirements(command.getRequirements());
        }
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
                command.end(true);
                activeCommands.remove(command);
            }
        }
    }

    @Override
    public void initialize() {
        activeCommands.clear();
        activeCommands.addAll(allCommands);
        activeCommands.add(deadline); // ensure deadline is included

        // Initialize all commands
        for (Command command : activeCommands) {
            command.initialize();
        }
    }

    @Override
    public void execute() {
        Iterator<Command> iterator = activeCommands.iterator();

        while (iterator.hasNext()) {
            Command command = iterator.next();

            // Skip if already finished
            if (command.isFinished()) {
                command.end(false);
                iterator.remove();
                continue;
            }

            command.execute();

            // If finished during execute
            if (command.isFinished()) {
                command.end(false);
                iterator.remove();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // The group finishes as soon as the deadline command finishes
        return deadline.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // Interrupt all remaining active commands
        for (Command command : activeCommands) {
            command.end(true);
        }
        activeCommands.clear();

        // Also end the deadline command if it’s still active
        if (!deadline.isFinished()) {
            deadline.end(true);
        }
    }
}
