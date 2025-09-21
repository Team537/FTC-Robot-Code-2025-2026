package org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class DeferredCommand extends Command {
    private final Supplier<Command> commandSupplier;
    private Command commandInstance;

    public DeferredCommand(Supplier<Command> commandSupplier, Subsystem... requirements) {
        this.commandSupplier = commandSupplier;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        commandInstance = commandSupplier.get();
        commandInstance.initialize();
    }

    @Override
    public void execute() {
        if (commandInstance != null) commandInstance.execute();
    }

    @Override
    public boolean isFinished() {
        return commandInstance != null && commandInstance.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (commandInstance != null) commandInstance.end(interrupted);
        commandInstance = null;
    }
}
