package org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

public class RepeatCommand extends Command {
    private final Command command;

    public RepeatCommand(Command command) {
        this.command = command;
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {

        // If the wrapped command finishes, re-initialize it immediately
        if (command.isFinished()) {
            command.end(false);
            command.initialize();
        }

        command.execute();

    }

    @Override
    public boolean isFinished() {
        return false; // repeats indefinitely until canceled
    }

    @Override
    public void end(boolean interrupted) {
        command.end(true);
    }
}
