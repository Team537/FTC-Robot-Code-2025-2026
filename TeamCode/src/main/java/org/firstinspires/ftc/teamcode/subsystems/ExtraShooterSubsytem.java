//Use this in case there is an extra motor, if there is not another motor, then use the Shooter Subsystem

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.ExtraShooterConfig;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class ExtraShooterSubsystem extends Subsystem {
    DcMotorEx firstShooterMotor;

    DcMotorEx secondShooterMotor;

    //Map the name and direction of both shooters
    public ExtraShooterSubsystem (HardwareMap hardwareMap, ExtraShooterConfig config) {
        super();

        firstShooterMotor = hardwareMap.get(DcMotorEx.class, config.motorName1);
        secondShooterMotor = hardwareMap.get(DcMotorEx.class, config.motorName2);

        firstShooterMotor.setDirection(config.motorDirection1);
        secondShooterMotor.setDirection(config.motorDirection2);
    }

    //Command for getting the overall velocity of both shooters
    public Command getVelocityCommand(Supplier<Double> velocitySupplier) {
        return new RunCommand(
                () -> {
                    firstShooterMotor.setPower(velocitySupplier.get() / Constants.ExtraShooter.MAX_VELOCITY);
                    secondShooterMotor.setPower(velocitySupplier.get() / Constants.ExtraShooter.MAX_VELOCITY);
                }
        ).withRequirements(this);
    }

    //Command for stopping both shooters
    public Command getStopVelocityCommand() {
        return new InstantCommand(
                () -> {
                    firstShooterMotor.setVelocity(0.0);
                    secondShooterMotor.setVelocity(0.0);
                }
        );
    }
}
