package org.firstinspires.ftc.teamcode.util.commandsystem.Commands;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

public class RunCommand extends Command {
    private final Runnable executeRunnable;

    public RunCommand(Runnable executeRunnable) {
        this.executeRunnable = executeRunnable;
    }

    @Override
    public void execute() {
        executeRunnable.run();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until canceled
    }
}
