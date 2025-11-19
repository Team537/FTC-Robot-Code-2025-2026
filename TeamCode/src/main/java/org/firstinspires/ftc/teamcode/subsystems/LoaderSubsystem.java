package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;

public class LoaderSubsystem {

    Servo loaderServo;

    public LoaderSubsystem(HardwareMap hardwareMap) {
        super();

        loaderServo = hardwareMap.get(Servo.class, Constants.Loader.LOADER_NAME);

        loaderServo.setPosition(Constants.Loader.START_POSITION);
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
        )
    }
}