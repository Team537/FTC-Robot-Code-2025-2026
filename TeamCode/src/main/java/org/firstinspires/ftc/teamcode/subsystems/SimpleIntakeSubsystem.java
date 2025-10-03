package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.FunctionalCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class SimpleIntakeSubsystem extends Subsystem {

    DcMotorEx intakeMotor;

    public SimpleIntakeSubsystem(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotorEx.class,"shooter");

    }

    public Command getIntakeCommand(Supplier<Double> power) {
        return new RunCommand(
            () -> {
                intakeMotor.setPower(power.get());
            }
        ).withRequirements(this);
    }

}
