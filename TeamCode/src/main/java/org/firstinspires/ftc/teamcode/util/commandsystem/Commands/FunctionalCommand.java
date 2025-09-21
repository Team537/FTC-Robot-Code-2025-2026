package org.firstinspires.ftc.teamcode.util.commandsystem.Commands;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

import java.util.function.BooleanSupplier;

public class FunctionalCommand extends Command {
    private final Runnable initRunnable;
    private final Runnable executeRunnable;
    private final BooleanSupplier isFinishedSupplier;
    private final Runnable endRunnable;

    public FunctionalCommand(Runnable initRunnable, Runnable executeRunnable,
                       BooleanSupplier isFinishedSupplier, Runnable endRunnable) {
        this.initRunnable = initRunnable;
        this.executeRunnable = executeRunnable;
        this.isFinishedSupplier = isFinishedSupplier;
        this.endRunnable = endRunnable;
    }

    @Override
    public void initialize() {
        if (initRunnable != null) initRunnable.run();
    }

    @Override
    public void execute() {
        if (executeRunnable != null) executeRunnable.run();
    }

    @Override
    public boolean isFinished() {
        return isFinishedSupplier != null && isFinishedSupplier.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        if (endRunnable != null) endRunnable.run();
    }
}
