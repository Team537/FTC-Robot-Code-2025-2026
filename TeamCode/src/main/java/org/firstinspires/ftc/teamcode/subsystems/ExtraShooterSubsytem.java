//Use this in case there is an extra motor, if there is not another motor, then use the Shooter Subsystem

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class ExtraShooterSubsytem extends Subsystem {
    DcMotorEx firstShooterMotor;

    DcMotorEx secondShooterMotor;

    //Map the name and direction of both shooters
    public ExtraShooterSubsytem (HardwareMap hardwareMap) {
        super();

        firstShooterMotor = hardwareMap.get(DcMotorEx.class, Constants.ExtraShooter.FIRST_SHOOTER_NAME);
        secondShooterMotor = hardwareMap.get(DcMotorEx.class, Constants.ExtraShooter.SECOND_SHOOTER_NAME);

        firstShooterMotor.setDirection(Constants.ExtraShooter.FIRST_SHOOTER_DIRECTION);
        secondShooterMotor.setDirection(Constants.ExtraShooter.SECOND_SHOOTER_DIRECTION);
    }

    //Command for getting the overall velocity of both shooters
    public Command getVelocityCommand(Supplier<Double> velocitySupplier) {
        return new RunCommand(
                () -> {
                    double wheelSpeed = velocitySupplier.get();
                    double ticksPerInch = Constants.ExtraShooter.TICKS_PER_REVOLUTION / Constants.ExtraShooter.WHEEL_CIRCUMFERENCE;
                    double wheelTicksPerInch = wheelSpeed * ticksPerInch;

                    firstShooterMotor.setVelocity(wheelTicksPerInch);
                    secondShooterMotor.setVelocity(wheelTicksPerInch);
                }
        );
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
