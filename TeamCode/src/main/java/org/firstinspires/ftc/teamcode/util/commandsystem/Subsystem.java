package org.firstinspires.ftc.teamcode.util.commandsystem;

public abstract class Subsystem {
    public void periodic() {}

    public void register() {
       CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void unregister() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    private Command defaultCommand = null;

    /** Assign a default command for this subsystem. */
    public void setDefaultCommand(Command command) {
        if (command.hasRequirement(this)) {
            this.defaultCommand = command;
        }
    }

    /** Remove the default command (none will run when idle). */
    public void clearDefaultCommand() {
        this.defaultCommand = null;
    }

    /** Get the currently set default command. */
    public Command getDefaultCommand() {
        return defaultCommand;
    }

}
