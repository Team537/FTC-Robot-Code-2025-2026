package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.routines.ExampleRoutine;
import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.MotorState;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

public class RobotContainer {

    public static RobotContainer instance;
    public OpMode opMode;

    public MecanumDriveSubsystem driveSubsystem;

    private final MotorSubsystem intakeSubsystem;

    private final MotorSubsystem hopperSubsystem;
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
        driveSubsystem = new MecanumDriveSubsystem(opMode.hardwareMap);
        driveSubsystem.register();
        intakeSubsystem = new MotorSubsystem(opMode.hardwareMap, Constants.Assemblys.intakeMotor);
        intakeSubsystem.register();
        hopperSubsystem = new MotorSubsystem(opMode.hardwareMap, Constants.Assemblys.hopperMotor);
        hopperSubsystem.register();
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
//                            if (translationalVelocity.norm() > Constants.Drive.MAX_TRANSLATIONAL_SPEED) {
//                                translationalVelocity = translationalVelocity.div(translationalVelocity.norm()).times(Constants.Drive.MAX_TRANSLATIONAL_SPEED);
//                            }

                            // Get the rotational velocity frame the gamepad.
                            // Inverted because left (negative X) should result in a counter-clockwise (positive) rotation
                            double rotationalVelocity = -gamepad1.right_stick_x * Constants.Drive.MAX_ROTATIONAL_SPEED;

                            // Clamp rotational velocity
//                            rotationalVelocity = MathUtil.clamp(rotationalVelocity, -Constants.Drive.MAX_ROTATIONAL_SPEED, Constants.Drive.MAX_ROTATIONAL_SPEED);

                            // Return a value of the combined velocities
                            return new ChassisVelocity2d(
                                    translationalVelocity,
                                    rotationalVelocity
                            );

                        }

                )
        );

        intakeSubsystem.setDefaultCommand(
                intakeSubsystem.getFeedCommand(

                        () -> {
                            boolean runMotorForward = gamepad1.b;
                            boolean runMotorBackward = gamepad1.a;
                            TelemetryManager.put("A Pressed", runMotorBackward);
                            TelemetryManager.put("B Pressed", runMotorForward);

                            MotorState determinedState = runMotorForward ? MotorState.Forward : runMotorBackward ? MotorState.Backward : MotorState.AtRest;

                            TelemetryManager.put("State", determinedState);


                            return determinedState;

                        }

                )
        );

        hopperSubsystem.setDefaultCommand(
                hopperSubsystem.getFeedCommand(

                        () -> {
                            boolean runMotorForward = gamepad1.x;
                            boolean runMotorBackward = gamepad1.y;
                            TelemetryManager.put("X Pressed", runMotorForward);
                            TelemetryManager.put("Y Pressed", runMotorBackward);

                            MotorState determinedState = runMotorForward ? MotorState.Forward : runMotorBackward ? MotorState.Backward : MotorState.AtRest;

                            TelemetryManager.put("State", determinedState);


                            return determinedState;

                        }

                )
        );

    }
    public void scheduleAuto() {
        TelemetryManager.put("Wheels move", true);
        ExampleRoutine routine = new ExampleRoutine(this.driveSubsystem, this.hopperSubsystem, this.intakeSubsystem);
        routine.getCommand().schedule();
        TelemetryManager.put("wheels stop", true);
    }



}


