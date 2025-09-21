package org.firstinspires.ftc.teamcode.util.commandsystem.Commands;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

import java.util.function.BooleanSupplier;

public class WaitCommand extends Command {
    private final long durationNanos;
    private final BooleanSupplier condition;
    private long startTime;

    // Wait for a fixed duration in seconds
    public WaitCommand(double seconds) {
        this.durationNanos = (long)(seconds * 1e9);
        this.condition = null;
    }

    // Wait for a condition to be true
    public WaitCommand(BooleanSupplier condition) {
        this.durationNanos = -1;
        this.condition = condition;
    }

    @Override
    public void initialize() {
        startTime = System.nanoTime();
    }

    @Override
    public boolean isFinished() {
        if (condition != null) {
            return condition.getAsBoolean();
        } else {
            return System.nanoTime() - startTime >= durationNanos;
        }
    }
}
