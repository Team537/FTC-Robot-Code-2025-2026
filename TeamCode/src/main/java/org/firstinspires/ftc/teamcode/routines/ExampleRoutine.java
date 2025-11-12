package org.firstinspires.ftc.teamcode.routines;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

public class ExampleRoutine {
    private MecanumDriveSubsystem driveSubsystem;
    private MotorSubsystem hopperSubsystem;
    private MotorSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public ExampleRoutine(MecanumDriveSubsystem driveSubsystem, MotorSubsystem hopperSubsystem, MotorSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    public Command getCommand() {
        return this.driveSubsystem.getDriveToPoseCommand(
                new Pose2d(new Translation2d(100, 100), new Rotation2d(0))
        ).andThen(this.shooterSubsystem.getAutoShootCommand());
    }
}
