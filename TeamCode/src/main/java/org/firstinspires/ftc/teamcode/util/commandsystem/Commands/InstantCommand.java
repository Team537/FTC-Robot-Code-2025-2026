package org.firstinspires.ftc.teamcode.util.commandsystem.Commands;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

public class InstantCommand extends Command {
    private final Runnable initRunnable;

    public InstantCommand(Runnable initRunnable) {
        this.initRunnable = initRunnable;
    }

    @Override
    public void initialize() {
        initRunnable.run();
    }

    @Override
    public boolean isFinished() {
        return true; // finishes immediately after init
    }
}
