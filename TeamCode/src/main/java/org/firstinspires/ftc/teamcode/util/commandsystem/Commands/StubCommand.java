package org.firstinspires.ftc.teamcode.util.commandsystem.Commands;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

public class StubCommand extends Command {

    boolean continuous;
    public StubCommand(boolean continuous) {
        this.continuous = continuous;
    }

    @Override
    public boolean isFinished() {
        return !continuous;
    }

}
