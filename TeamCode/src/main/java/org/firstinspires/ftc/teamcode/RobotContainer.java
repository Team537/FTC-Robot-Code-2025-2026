package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.FunctionalCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

public class RobotContainer {

    public static RobotContainer instance;
    public OpMode opMode;

    public MecanumDriveSubsystem driveSubsystem;

    public static RobotContainer getInstance(OpMode opMode) {
        if (instance == null) {
            instance = new RobotContainer(opMode);
        }
        if (opMode != instance.opMode) {
            instance.opMode = opMode;
        }
        return instance;
    }

    private RobotContainer(OpMode opMode) {
        this.opMode = opMode;
        driveSubsystem = new MecanumDriveSubsystem(opMode.hardwareMap);
        driveSubsystem.register();
    }

    /**
     * returns a gamepad from the active opMode
     * @param number the ID of the gamepad (1 or 2)
     * @return the gamepad of the active opMode
     */
    private Gamepad getGamepad(int number) {
        switch (number) {
            case 1:
                return opMode.gamepad1;
            case 2:
                return opMode.gamepad2;
            default:
                throw new IllegalArgumentException("Expected controller 1 or 2");
        }
    }

    /**
     * schedule all commands for manual control
     */
    public void scheduleTeleOp() {
        driveSubsystem.setDefaultCommand(
            driveSubsystem.getDriveVelocityCommand(

                () -> {
                    Translation2d translationalVelocity = new Translation2d(
                        getGamepad(1).left_stick_x * Constants.Drive.MAX_TRANSLATIONAL_SPEED,
                        -getGamepad(1).left_stick_y * Constants.Drive.MAX_TRANSLATIONAL_SPEED
                    );

                    if (translationalVelocity.norm() > Constants.Drive.MAX_TRANSLATIONAL_SPEED) {
                        translationalVelocity = translationalVelocity.div(translationalVelocity.norm()).times(Constants.Drive.MAX_TRANSLATIONAL_SPEED);
                    }

                    double rotationalVelocity = -getGamepad(1).right_stick_x * Constants.Drive.MAX_ROTATIONAL_SPEED;

                    rotationalVelocity = MathUtil.clamp(rotationalVelocity, -Constants.Drive.MAX_ROTATIONAL_SPEED, Constants.Drive.MAX_ROTATIONAL_SPEED);

                    return new ChassisVelocity2d(
                        translationalVelocity,
                        rotationalVelocity
                    );

                }




            )
        );

    }

}
