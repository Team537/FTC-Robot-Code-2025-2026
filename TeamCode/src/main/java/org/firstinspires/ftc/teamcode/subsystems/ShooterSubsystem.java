package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MotorState;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class ShooterSubsystem extends Subsystem {
    private final MotorSubsystem leftMotorSubsystem;
    private final MotorSubsystem rightMotorSubsystem;

    public ShooterSubsystem (HardwareMap hardwareMap, String leftMotorName, String rightMotorName) {
        super();

        this.leftMotorSubsystem = new MotorSubsystem(hardwareMap, leftMotorName);
        this.rightMotorSubsystem = new MotorSubsystem(hardwareMap, rightMotorName);
        this.rightMotorSubsystem.setMotorInversion(true);
    }
    public Command getManualShootCommand(Supplier<MotorState> motorStateSupplier) {
        return new ParallelCommandGroup(
            this.leftMotorSubsystem.getManualCommand(motorStateSupplier),
            this.rightMotorSubsystem.getManualCommand(motorStateSupplier)
        ).withRequirements(this);
    }

    public Command getVelocityShootCommand(Supplier<Double> rpm) {
        return new ParallelCommandGroup(
                this.leftMotorSubsystem.getVelocityCommand(rpm),
                this.rightMotorSubsystem.getVelocityCommand(rpm)
        ).withRequirements(this);
    }

    public Command getAutoShootCommand() {
        return new ParallelCommandGroup(
            this.leftMotorSubsystem.getRunForTimeCommand(4, MotorState.Forward),
            this.rightMotorSubsystem.getRunForTimeCommand(4, MotorState.Backward)
        ).withRequirements(this);
    }
}
