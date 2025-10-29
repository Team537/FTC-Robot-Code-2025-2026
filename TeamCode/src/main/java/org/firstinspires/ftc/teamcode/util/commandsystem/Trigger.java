package org.firstinspires.ftc.teamcode.util.commandsystem;

import java.util.function.BooleanSupplier;

public class Trigger {
    private final BooleanSupplier condition;
    private boolean lastState = false;

    public Trigger(BooleanSupplier condition) {
        this.condition = condition;
    }

    // Called every loop
    public void update() {
        boolean current = condition.getAsBoolean();

        // Rising edge = pressed
        if (current && !lastState && onPress != null) {
            onPress.schedule();
        }

        // While held
        if (current && !lastState && whileHeld != null) {
            whileHeld.schedule();
        }

        if (!current && lastState && whileHeld != null) {
            whileHeld.cancel();
        }

        // Falling edge = released
        if (!current && lastState && onRelease != null) {
            onRelease.schedule();
        }

        lastState = current;
    }

    // Command bindings
    private Command onPress, onRelease, whileHeld;

    public Trigger onPress(Command command) {
        this.onPress = command;
        return this;
    }

    public Trigger whileHeld(Command command) {
        this.whileHeld = command;
        return this;
    }

    public Trigger onRelease(Command command) {
        this.onRelease = command;
        return this;
    }

    public void add() {
        CommandScheduler.getInstance().addTrigger(this);
    }

    public void remove() {
        CommandScheduler.getInstance().removeTrigger(this);
    }
}
