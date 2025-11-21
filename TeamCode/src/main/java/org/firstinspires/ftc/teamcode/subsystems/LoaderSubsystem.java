package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.MotorState;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class LoaderSubsystem extends Subsystem {

    Servo loaderServo;

    public LoaderSubsystem(HardwareMap hardwareMap) {
        super();

        loaderServo = hardwareMap.get(Servo.class, Constants.Loader.LOADER_NAME);
        loaderServo.setPosition(Constants.Loader.LOAD_POSITION);
    }

    public Command setLoadPositionCommand() {
        return new InstantCommand(
                () -> {
                    loaderServo.setPosition(Constants.Loader.LOAD_POSITION);
                }
        );
    }

    public Command setShootPositionCommand() {
        return new InstantCommand(
                () -> {
                    loaderServo.setPosition(Constants.Loader.SHOOT_POSITION);
                }
        );
    }

    public Command getManualCommand(Supplier<MotorState> motorStateSupplier) {
        return new RunCommand(() -> {
            double position = 0.9;
            if (motorStateSupplier.get() == MotorState.Forward) {
                position = 1;
            }

            this.loaderServo.setPosition(position);
            TelemetryManager.put("position", position);
        }).withRequirements(this);
    }
}
