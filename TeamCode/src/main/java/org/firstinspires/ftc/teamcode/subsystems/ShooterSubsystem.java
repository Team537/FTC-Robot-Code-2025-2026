//Use this if the robot has only one shooter, use the Extra Shooter Subsystem if there are two shooters
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.MAX_VELOCITY;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class ShooterSubsystem extends Subsystem {

    //Determine motor for the Shooter
    private DcMotorEx shooterMotor;
    //Call configuration for shooter
    private ShooterConfig config;

    // Maps the direction and name of the motor
    public ShooterSubsystem (HardwareMap hardwareMap, ShooterConfig config) {
        super();
        this.config = config;

        shooterMotor = hardwareMap.get(DcMotorEx.class, config.motorName);

        shooterMotor.setDirection(config.motorDirection);

        shooterMotor.setVelocityPIDFCoefficients(1,0,0,15);
    }

    //Command to get the Velocity
    public Command getSetVelocityCommand(Supplier<Double> velocitySupplier) {
        return new RunCommand(
                () -> {
                    double wheelSpeed = velocitySupplier.get();
                    double ticksPerInch = Constants.Shooter.TICKS_PER_REVOLUTION / Constants.Shooter.WHEEL_CIRCUMFERENCE;
                    double wheelSpeedTicksPerSec = wheelSpeed * ticksPerInch;

                    shooterMotor.setVelocity(wheelSpeedTicksPerSec);
                }
        ).withRequirements(this);
    }

    //Command to stop the velocity
    public Command getStopVelocityCommand() {
        return new InstantCommand(
                () -> {
                    shooterMotor.setVelocity(0.0);
        }
        );
    }


}
