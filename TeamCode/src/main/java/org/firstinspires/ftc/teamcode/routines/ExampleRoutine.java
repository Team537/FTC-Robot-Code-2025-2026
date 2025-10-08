package org.firstinspires.ftc.teamcode.routines;

import org.firstinspires.ftc.teamcode.subsystems.HolonomicDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

public class ExampleRoutine {
    private HolonomicDriveSubsystem driveSubsystem;
    private MotorSubsystem hopperSubsystem;
    private MotorSubsystem intakeSubsystem;
    public ExampleRoutine(HolonomicDriveSubsystem driveSubsystem, MotorSubsystem hopperSubsystem, MotorSubsystem intakeSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command getCommand() {
        return this.driveSubsystem.getDriveToPoseCommand(new Pose2d(new Translation2d(10, 10), new Rotation2d(0)));
    }

}
