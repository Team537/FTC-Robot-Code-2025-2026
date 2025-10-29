package org.firstinspires.ftc.teamcode.util.commandsystem;

import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

public class CommandScheduler {

    private static CommandScheduler INSTANCE;

    private CommandScheduler() {}

    public static CommandScheduler getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CommandScheduler();
        }
        return INSTANCE;
    }

    private final Set<Subsystem> subsystems = new LinkedHashSet<>();
    private final Map<Subsystem, Command> requirements = new HashMap<>();

    /** Register a subsystem with the scheduler. */
    public void registerSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    /** Remove a subsystem from the scheduler. */
    public void unregisterSubsystem(Subsystem subsystem) {
        subsystems.remove(subsystem);
        requirements.remove(subsystem);
    }

    public void schedule(Command command) {

        // cancel conflicting commands
        for (Subsystem requirement : command.getRequirements()) {
            if (requirements.containsKey(requirement)) {
                cancel(requirements.get(requirement));
            }
        }
        if (!commands.contains(command)) {
            pendingCommands.add(command);
        }

    }

    public void cancel(Command command) {
        if (commands.contains(command) && !endingCommands.contains(command)) {
            command.end(true);
            endingCommands.add(command);
            for (Subsystem requirement : command.getRequirements()) {
                requirements.remove(requirement);
            }
        }
    }

    public void cancelAll() {
        for (Command command : commands) {
            command.cancel();
        }
    }

    public void addTrigger(Trigger trigger) {
        triggers.add(trigger);
    }
    public void removeTrigger(Trigger trigger) {
        triggers.remove(trigger);
    }

    private Set<Command> commands = new LinkedHashSet<>();
    private Set<Command> pendingCommands = new LinkedHashSet<>();
    private Set<Command> endingCommands = new LinkedHashSet<>();
    private Set<Trigger> triggers = new LinkedHashSet<>();

    public void run() {

        // run all subsystem periodic methods
        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }

        // Update triggers (check inputs)
        for (Trigger t : triggers) {
            t.update();
        }

        //scheduling all pending commands at one time so scheduling while commands are running doesn't cause any issues.
        for (Command pendingCommand : pendingCommands) {
            //changing the command from the pending list to the command list
            commands.add(pendingCommand);

            //adding requirements
            for (Subsystem requirement : pendingCommand.getRequirements()) {
                requirements.put(requirement, pendingCommand);
            }

            //running the initialize method on the command
            pendingCommand.initialize();
        }
        pendingCommands.clear();

        //running all commands
        for (Command command : commands) {
            if (!endingCommands.contains(command)) {
                if (command.isFinished()) {
                    command.end(false);
                    endingCommands.add(command);
                    for (Subsystem requirement : command.getRequirements()) {
                        requirements.remove(requirement);
                    }
                    continue;
                }
                command.execute();
            }
        }

        for (Command endingCommand : endingCommands) {
            commands.remove(endingCommand);
        }
        endingCommands.clear();

        // run default commands if subsystems are idle
        for (Subsystem subsystem : subsystems) {
            if (!requirements.containsKey(subsystem)) {
                Command defaultCommand = subsystem.getDefaultCommand();
                if (defaultCommand != null) {
                    schedule(defaultCommand);
                }
            }
        }

    }

}
