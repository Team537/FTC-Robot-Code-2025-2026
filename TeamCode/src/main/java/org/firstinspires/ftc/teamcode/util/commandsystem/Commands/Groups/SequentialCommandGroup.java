package org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A command group that runs commands sequentially, one after another.
 */
public class SequentialCommandGroup extends Command {

    /** All commands that belong to this group (permanent structure). */
    private final List<Command> allCommands = Collections.synchronizedList(new ArrayList<>());

    /** Index of the currently active command. */
    private int currentIndex = 0;

    /** Current active command reference. */
    private Command currentCommand = null;

    public SequentialCommandGroup(List<Command> initialCommands) {
        add(initialCommands);
    }

    public SequentialCommandGroup(Command... initialCommands) {
        add(initialCommands);
    }

    /** Add commands to the group (varargs). */
    public void add(Command... commands) {
        add(List.of(commands));
    }

    /** Add commands to the group (List). */
    public void add(List<Command> commands) {
        allCommands.addAll(commands);
    }

    /** Remove commands permanently from the group (varargs). */
    public void remove(Command... commands) {
        remove(List.of(commands));
    }

    /** Remove commands permanently from the group (List). */
    public void remove(List<Command> commands) {
        for (Command command : commands) {
            int index = allCommands.indexOf(command);
            if (index >= 0) {
                // If the command to remove is currently running, end it as interrupted
                if (currentCommand == command) {
                    currentCommand.end(true);
                    currentCommand = null;
                }
                allCommands.remove(command);

                // Adjust currentIndex if needed
                if (index < currentIndex) {
                    currentIndex--;
                }
            }
        }
    }

    @Override
    public void initialize() {
        // Start from the first command
        currentIndex = 0;
        if (!allCommands.isEmpty()) {
            currentCommand = allCommands.get(currentIndex);
            currentCommand.initialize();
        } else {
            currentCommand = null;
        }
    }

    @Override
    public void execute() {
        if (currentCommand == null) {
            return; // no commands left
        }

        // Check if finished BEFORE executing
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            advanceToNextCommand();
            return;
        }

        // Otherwise, run normally
        currentCommand.execute();

        // Check again after execution in case it finishes mid-tick
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            advanceToNextCommand();
        }
    }

    /** Advance to the next command in the sequence. */
    private void advanceToNextCommand() {
        currentIndex++;
        if (currentIndex < allCommands.size()) {
            currentCommand = allCommands.get(currentIndex);
            currentCommand.initialize();
        } else {
            currentCommand = null; // finished sequence
        }
    }

    @Override
    public boolean isFinished() {
        return currentCommand == null;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(true);
        }
        currentCommand = null;
        currentIndex = 0;
    }
}