package org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

import java.util.function.BooleanSupplier;

public class ConditionalCommand extends Command {
    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;

    private Command selected;

    public ConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.condition = condition;
    }

    @Override
    public void initialize() {
        selected = condition.getAsBoolean() ? onTrue : onFalse;
        selected.initialize();
    }

    @Override
    public void execute() {
        selected.execute();
    }

    @Override
    public boolean isFinished() {
        return selected.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        selected.end(interrupted);
    }
}
