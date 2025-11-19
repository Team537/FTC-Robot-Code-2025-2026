package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.routines.ExampleRoutine;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.MotorState;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Trigger;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

public class RobotContainer {
    public static RobotContainer instance;

    public OpMode opMode;

    public MecanumDriveSubsystem driveSubsystem;

    private final MotorSubsystem intakeSubsystem;

    private final MotorSubsystem hopperSubsystem;

    private final ShooterSubsystem shooterSubsytem;

    private final LoaderSubsystem loaderSubsystem;

    private Gamepad gamepad1;

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
        this.driveSubsystem = new MecanumDriveSubsystem(opMode.hardwareMap, Constants.Drive.MECANUM_DRIVE_CONFIG);
        this.driveSubsystem.register();
        this.intakeSubsystem = new MotorSubsystem(opMode.hardwareMap, Constants.Assemblys.intakeMotor);
        this.intakeSubsystem.register();
        this.hopperSubsystem = new MotorSubsystem(opMode.hardwareMap, Constants.Assemblys.hopperMotor);
        this.hopperSubsystem.register();
        this.shooterSubsytem = new ShooterSubsystem(opMode.hardwareMap, Constants.Assemblys.shooterMotorLeft, Constants.Assemblys.shooterMotorRight);
        this.shooterSubsytem.register();
        this.loaderSubsystem = new LoaderSubsystem(opMode.hardwareMap);
        this.loaderSubsystem.register();
        bindGamepads(opMode);
    }

    /**
     * Binds the controller objects to the respective controllers.
     * This is needed because the controller objects do not save across opModes.
     * @param opMode
     */
    private void bindGamepads(OpMode opMode) {
        gamepad1 = opMode.gamepad1;
    }

    public void setupHardware() {
        driveSubsystem.setupMotors();
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
//                            if (translationalVelocity.norm() > Constants.Drive.MAX_TRANSLATIONAL_SPEED) {
//                                translationalVelocity = translationalVelocity.div(translationalVelocity.norm()).times(Constants.Drive.MAX_TRANSLATIONAL_SPEED);
//                            }

                            // Get the rotational velocity frame the gamepad.
                            // Inverted because left (negative X) should result in a counter-clockwise (positive) rotation
                            double rotationalVelocity = -gamepad1.right_stick_x * Constants.Drive.MAX_ROTATIONAL_SPEED;

                            // Clamp translational velocity
                            if (translationalVelocity.magnitude() > Constants.Drive.MAX_TRANSLATIONAL_SPEED) {
                                translationalVelocity = translationalVelocity.div(translationalVelocity.magnitude()).times(Constants.Drive.MAX_TRANSLATIONAL_SPEED);
                            }

                            // Return a value of the combined velocities
                            return new ChassisVelocity2d(translationalVelocity, rotationalVelocity);
                        }
                )
        );

        intakeSubsystem.setDefaultCommand(
                intakeSubsystem.getManualCommand(
                        () -> {
                            boolean intakePressed = gamepad1.right_bumper;
                            boolean reversePressed = gamepad1.left_trigger <= 0;
                            TelemetryManager.put("Run Intake", intakePressed);
                            TelemetryManager.put("Reverse", reversePressed);

                            MotorState determinedState = intakePressed ? reversePressed ? MotorState.Backward : MotorState.Forward : MotorState.AtRest;
                            TelemetryManager.put("Intake State", determinedState);

                            return determinedState;
                        }
                )
        );

        hopperSubsystem.setDefaultCommand(
                hopperSubsystem.getManualCommand(
                        () -> {
                            boolean hopperPressed = gamepad1.right_bumper;
                            boolean reversePressed = gamepad1.left_trigger > 0;
                            TelemetryManager.put("Run Hopper", hopperPressed);
                            TelemetryManager.put("Reverse", reversePressed);

                            MotorState determinedState = hopperPressed ? reversePressed ? MotorState.Backward : MotorState.Forward : MotorState.AtRest;
                            TelemetryManager.put("Hopper State", determinedState);

                            return determinedState;
                        }
                )
        );


        shooterSubsytem.setDefaultCommand(
                shooterSubsytem.getManualShootCommand(
                        () -> {
                            boolean shooterPressed = gamepad1.x;
                            boolean reversePressed = gamepad1.left_trigger > 0;
                            TelemetryManager.put("Run Shooter", shooterPressed);
                            TelemetryManager.put("Reverse", reversePressed);

                            MotorState determinedState = shooterPressed ? reversePressed ? MotorState.Backward : MotorState.Forward : MotorState.AtRest;
                            TelemetryManager.put("Shooter State", determinedState);

                            return determinedState;
                        }
                )
        );

//        loaderSubsystem.setDefaultCommand(
//                loaderSubsystem.getManualCommand(
//                        () -> {
//                            boolean releasePressed = gamepad1.a;
//                            TelemetryManager.put("Release Shooter", releasePressed);
//
//                            MotorState determinedState = releasePressed ? MotorState.Forward : MotorState.AtRest;
//                            TelemetryManager.put("Release State", determinedState);
//
//                            return determinedState;
//                        }
//                )
//        );

        Trigger shootTrigger = new Trigger(() -> gamepad1.a);
        shootTrigger.onPress(loaderSubsystem.setShootPositionCommand());
        shootTrigger.onRelease(loaderSubsystem.setLoadPositionCommand());
        shootTrigger.add();
    }

    public void scheduleAuto() {
        TelemetryManager.put("Scheduling Auto", true);
        ExampleRoutine routine = new ExampleRoutine(this.driveSubsystem, this.hopperSubsystem, this.intakeSubsystem, this.shooterSubsytem);
        routine.getCommand().schedule();
        TelemetryManager.put("Scheduled Auto", true);
    }

    public void resetPose() {
        Command.instant(() -> driveSubsystem.setRobotPose(Pose2d.ZERO)).schedule();
    }

}
