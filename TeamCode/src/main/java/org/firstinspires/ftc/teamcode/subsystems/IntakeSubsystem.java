package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.IntakeConfig;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class IntakeSubsystem extends Subsystem {
    //Determine motor for intake subsystem
    DcMotorEx intakeMotor;
    private IntakeConfig config;

    //Define the motor name, direction, and the velocity PIDF coefficients
    public IntakeSubsystem (HardwareMap hardwareMap, IntakeConfig config) {
        super();

        intakeMotor = hardwareMap.get(DcMotorEx.class, config.motorName);

        intakeMotor.setDirection(config.motorDirection);

        intakeMotor.setVelocityPIDFCoefficients(1,0,0,15);
    }

    //Creates a command that determines the velocity of the intake motor
    public Command getSetVelocityCommand(Supplier<Double> velocitySupplier) {
        return new RunCommand(
                () -> {
                    double wheelSpeed = velocitySupplier.get();
                    double ticksPerInch = Constants.Intake.TICKS_PER_REVOLUTION / Constants.Intake.WHEEL_CIRCUMFERENCE;
                    double wheelSpeedTicksPerSec = wheelSpeed * ticksPerInch;

                    intakeMotor.setVelocity(wheelSpeedTicksPerSec);
                }
        ).withRequirements(this);
    }

    //Creates a command that stops the intake motor
    public Command getStopVeloctyCommand() {
        return new InstantCommand(
                () -> {
                    intakeMotor.setVelocity(0.0);
                }
        );
    }
}
