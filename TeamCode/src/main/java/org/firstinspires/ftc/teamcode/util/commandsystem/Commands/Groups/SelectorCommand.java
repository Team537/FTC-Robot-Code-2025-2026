package org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;

import java.util.Map;
import java.util.function.Supplier;

public class SelectorCommand<K> extends Command {
    private final Supplier<K> keySupplier;
    private final Map<K, Command> commandMap;

    private Command selected;

    public SelectorCommand(Supplier<K> keySupplier, Map<K, Command> commandMap) {
        this.keySupplier = keySupplier;
        this.commandMap = commandMap;
    }

    @Override
    public void initialize() {
        K key = keySupplier.get();
        selected = commandMap.get(key);
        if (selected != null) selected.initialize();
    }

    @Override
    public void execute() {
        if (selected != null) selected.execute();
    }

    @Override
    public boolean isFinished() {
        return selected == null || selected.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (selected != null) selected.end(interrupted);
    }
}
