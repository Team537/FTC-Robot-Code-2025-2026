package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DifferentialDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtraShooterSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ExtraShooterConfig;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

public class RobotContainer {

    public static RobotContainer instance;
    public OpMode opMode;

   public MecanumDriveSubsystem driveSubsystem;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

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
        //driveSubsystem = new MecanumDriveSubsystem(opMode.hardwareMap);
        //driveSubsystem.register();
        bindGamepads(opMode);
    }

    /**
     * Binds the controller objects to the respective controllers.
     * This is needed because the controller objects do not save across opModes.
     * @param opMode
     */
    private void bindGamepads(OpMode opMode) {
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
    }

    /**
     * schedule all commands for manual control
     */
    public void scheduleTeleOp() {
        driveSubsystem.setDefaultCommand(
            driveSubsystem.getDriveVelocityCommand(

                () -> {

                    // Get the translational velocities from the gamepad.
                    // Y is inverted because the gamepad reports "up" on the gamepad as negative, which is opposite to the coordinate frame we are using.
                    Translation2d translationalVelocity = new Translation2d(
                        gamepad1.left_stick_x * Constants.Drive.MAX_TRANSLATIONAL_SPEED,
                        -gamepad1.left_stick_y * Constants.Drive.MAX_TRANSLATIONAL_SPEED
                    );

                    // Clamp translational velocity
                    if (translationalVelocity.magnitude() > Constants.Drive.MAX_TRANSLATIONAL_SPEED) {
                        translationalVelocity = translationalVelocity.div(translationalVelocity.magnitude()).times(Constants.Drive.MAX_TRANSLATIONAL_SPEED);
                    }

                    // Get the rotational velocity frame the gamepad.
                    // Inverted because left (negative X) should result in a counter-clockwise (positive) rotation
                    double rotationalVelocity = -gamepad1.right_stick_x * Constants.Drive.MAX_ROTATIONAL_SPEED;

                    // Clamp rotational velocity
                    rotationalVelocity = MathUtil.clamp(rotationalVelocity, -Constants.Drive.MAX_ROTATIONAL_SPEED, Constants.Drive.MAX_ROTATIONAL_SPEED);

                    // Return a value of the combined velocities
                    return new ChassisVelocity2d(
                        translationalVelocity,
                        rotationalVelocity
                    );

                }

            )
        );

    }

}
