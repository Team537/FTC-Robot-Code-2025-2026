package org.firstinspires.ftc.teamcode.routines;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

public class ExampleRoutine {
    private MecanumDriveSubsystem driveSubsystem;
    private MotorSubsystem hopperSubsystem;
    private MotorSubsystem intakeSubsystem;
    private Command shootCommand;

    public ExampleRoutine(MecanumDriveSubsystem driveSubsystem, MotorSubsystem hopperSubsystem, MotorSubsystem intakeSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command getCommand() {
        Command driveToPose = this.driveSubsystem.getDriveToPoseCommand(
                new Pose2d(new Translation2d(1000, 1000), new Rotation2d(0))
        );
        
        Command setPositionAndShoot = new InstantCommand(() -> {
        }).andThen(shootCommand);
        return driveToPose.andThen(setPositionAndShoot);
    }

    
}
