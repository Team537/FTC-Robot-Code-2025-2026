package org.firstinspires.ftc.teamcode.util.commandsystem;

import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.FunctionalCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.ConditionalCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.DeferredCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.ParallelDeadlineGroup;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.ParallelRaceGroup;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.RepeatCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.SelectorCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.StartEndCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.StubCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.WaitCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;

import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class Command {

    public final void schedule() {
        CommandScheduler.getInstance().schedule(this);
    }

    public final void cancel() {
        CommandScheduler.getInstance().cancel(this);
    }

    public boolean isFinished() {
        return false;
    }

    public void execute() {}
    public void initialize() {}
    public void end(boolean interrupted) {}

    private final Set<Subsystem> requirements = new HashSet<>();

    /** Register a subsystem requirement. */
    protected void addRequirements(Subsystem... subsystems) {
        Collections.addAll(requirements, subsystems);
    }

    /** Register a subsystem requirement. */
    protected void addRequirements(Set<Subsystem> subsystems) {
        addRequirements(subsystems.toArray(new Subsystem[0]));
    }

    /** Get all subsystem requirements. */
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    /** Check if the command requires a subsystem. */
    public boolean hasRequirement(Subsystem subsystem) {
        return this.getRequirements().contains(subsystem);
    }

    public Command withRequirements(Subsystem... subsystems) {
        addRequirements(subsystems);
        return this;
    }

    /**
     * Set a Runnable to run at the start of the command.
     * @param runnable Runnable to run on initialize
     * @return this command for chaining
     */
    public SequentialCommandGroup onStart(Runnable runnable) {
        return new SequentialCommandGroup(instant(runnable), this);
    }

    /**
     * Set a Runnable to run at the end of the command.
     * @param runnable Runnable to run on end
     * @return this command for chaining
     */
    public SequentialCommandGroup finallyDo(Runnable runnable) {
        return new SequentialCommandGroup(this, instant(runnable));
    }

    /**
     * Set a Runnable to run along with the command.
     * @param runnable Runnable to run along with
     * @return this command for chaining
     */

    public ParallelCommandGroup alongWith(Runnable runnable) {
        return new ParallelCommandGroup(this, run(runnable));
    }

    // =============================
    // ======= Combinators =========
    // =============================

    /** Run this command in parallel with others. */
    public ParallelCommandGroup alongWith(Command... commands) {
        ParallelCommandGroup group = parallel(this);
        group.add(commands);
        return group;
    }

    /** Run this command in a race with others (group finishes when any command finishes). */
    public ParallelRaceGroup raceWith(Command... commands) {
        ParallelRaceGroup group = race(this);
        group.add(commands);
        return group;
    }

    /** Run this command with a deadline (this command is the deadline). */
    public ParallelDeadlineGroup deadlineFor(Command... commands) {
        ParallelDeadlineGroup group = deadline(this);
        group.add(commands);
        return group;
    }

    /** Run this command with another command as the deadline. */
    public ParallelDeadlineGroup withDeadline(Command deadline) {
        return deadline(deadline, this);
    }

    /** Run this command, then follow with additional commands sequentially. */
    public SequentialCommandGroup andThen(Command... commands) {
        SequentialCommandGroup group = sequential(this);
        group.add(commands);
        return group;
    }

    /** Run this command in parallel with a wait until a condition is true. */
    public ParallelRaceGroup until(BooleanSupplier condition) {
        return this.raceWith(waitUntil(condition));
    }

    /** Run this command only while a condition remains true. */
    public ParallelRaceGroup onlyWhile(BooleanSupplier condition) {
        return this.until(() -> !condition.getAsBoolean());
    }

    /** Run this command in parallel with a timeout. */
    public ParallelRaceGroup withTimeout(double seconds) {
        return this.raceWith(waitSeconds(seconds));
    }

    /** Run this command only if a condition is true. */
    public ConditionalCommand onlyIf(BooleanSupplier condition) {
        return Command.conditional(this, stub(), condition);
    }

    /** Run this command unless a condition is true. */
    public ConditionalCommand unless(BooleanSupplier condition) {
        return Command.conditional(stub(), this, condition);
    }

    /** Repeat this command indefinitely. */
    public RepeatCommand repeatedly() {
        return repeat(this);
    }

    // ======== Group Commands ========

    public static ParallelCommandGroup parallel(Command... commands) {
        return new ParallelCommandGroup(commands);
    }

    public static ParallelRaceGroup race(Command... commands) {
        return new ParallelRaceGroup(commands);
    }

    public static ParallelDeadlineGroup deadline(Command deadline, Command... commands) {
        ParallelDeadlineGroup group = new ParallelDeadlineGroup(deadline);
        group.add(commands);
        return group;
    }

    public static SequentialCommandGroup sequential(Command... commands) {
        return new SequentialCommandGroup(commands);
    }

    public static ConditionalCommand conditional(Command onTrue, Command onFalse, BooleanSupplier condition) {
        return new ConditionalCommand(onTrue, onFalse, condition);
    }

    public static <K> Command selector(Supplier<K> keySupplier, Map<K, Command> commandMap) {
        return new SelectorCommand<>(keySupplier, commandMap);
    }

    public static RepeatCommand repeat(Command command) {
        return new RepeatCommand(command);
    }

    public static DeferredCommand deferred(Supplier<Command> commandSupplier) {
        return new DeferredCommand(commandSupplier);
    }

    // ======== Functional / Instant Commands ========

    public static Command functional(
        Runnable init,
        Runnable execute,
        BooleanSupplier isFinished,
        Runnable end
    ) {
        return new FunctionalCommand(init, execute, isFinished, end);
    }

    public static Command instant(Runnable init) {
        return new InstantCommand(init);
    }

    public static Command run(Runnable execute) {
        return new RunCommand(execute);
    }

    public static Command startEnd(Runnable start, Runnable end) {
        return new StartEndCommand(start, end);
    }

    public static Command waitSeconds(double seconds) {
        return new WaitCommand(seconds);
    }

    public static Command waitUntil(BooleanSupplier condition) {
        return new WaitCommand(condition);
    }

    // ======== Stub / Empty Commands ========

    /** Ends immediately, does nothing */
    public static Command stub() {
        return new StubCommand(false);
    }

    /** Runs indefinitely until interrupted */
    public static Command continuousStub() {
        return new StubCommand(true);
    }

}
