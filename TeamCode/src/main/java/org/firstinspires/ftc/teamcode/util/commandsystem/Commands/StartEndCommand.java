package org.firstinspires.ftc.teamcode.util.commandsystem.Commands;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

public class StartEndCommand extends Command {
    private final Runnable startRunnable;
    private final Runnable endRunnable;

    public StartEndCommand(Runnable startRunnable, Runnable endRunnable) {
        this.startRunnable = startRunnable;
        this.endRunnable = endRunnable;
    }

    @Override
    public void initialize() {
        startRunnable.run();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until canceled
    }

    @Override
    public void end(boolean interrupted) {
        endRunnable.run();
    }
}
