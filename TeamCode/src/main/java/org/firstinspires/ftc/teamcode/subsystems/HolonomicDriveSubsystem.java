package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

import java.util.function.Supplier;

/**
 * Abstract base class for any holonomic (e.g., mecanum or omniwheel) drivetrain.
 * Provides:
 * - Separate translational and rotational subsystems
 * - PID-controlled driving to positions
 * - Velocity-based commands for driving
 * - Field-relative drive support
 */
public abstract class HolonomicDriveSubsystem extends Subsystem {

    /** Subsystem handling translational (X/Y) velocity */
    private TranslationalSubsystem translationalSubsystem;

    /** Subsystem handling rotational (yaw) velocity */
    private RotationalSubsystem rotationalSubsystem;

    /**
     * Inner subsystem representing translational velocity state.
     */
    private class TranslationalSubsystem extends Subsystem {
        private Translation2d translationalVelocity = new Translation2d(0.0,0.0);

        /** Set current translational velocity */
        public void setTranslationalVelocity(Translation2d velocity) {
            this.translationalVelocity = velocity;
        }

        /** Get current translational velocity */
        public Translation2d getTranslationalVelocity() {
            return translationalVelocity;
        }
    }

    /**
     * Inner subsystem representing rotational velocity state.
     */
    private class RotationalSubsystem extends Subsystem {
        private double rotationalVelocity = 0.0;

        /** Set current rotational velocity */
        public void setRotationalVelocity(double velocity) {
            this.rotationalVelocity = velocity;
        }

        /** Get current rotational velocity */
        public double getRotationalVelocity() {
            return rotationalVelocity;
        }
    }

    /** Current pose of the robot in field coordinates */
    private Pose2d robotPose = new Pose2d(new Translation2d(0.0,0.0),new Rotation2d(0.0));

    /** Set robot pose manually (e.g., for odometry reset) */
    public void setRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    /** Get the current robot pose */
    public Pose2d getRobotPose() {
        return robotPose;
    }

    /** PID controllers for X, Y translation and rotation */
    private PIDController xController = new PIDController(0.0,0.0,0.0);
    private PIDController yController = new PIDController(0.0,0.0,0.0);
    private PIDController thetaController = new PIDController(0.0,0.0,0.0);

    /** Constructor initializes subsystems and enables continuous input for rotation PID */
    public HolonomicDriveSubsystem() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        translationalSubsystem = new TranslationalSubsystem();
        rotationalSubsystem = new RotationalSubsystem();
    }

    /**
     * Periodic update called by scheduler.
     * Combines translational and rotational velocities into a chassis velocity,
     * applies field-relative rotation, sets motors, and updates odometry.
     */
    @Override
    public void periodic() {
        ChassisVelocity2d targetVelocity = new ChassisVelocity2d(
            translationalSubsystem.getTranslationalVelocity().rotateBy(getRobotPose().getRotation()),
            rotationalSubsystem.getRotationalVelocity()
        );
        setMotors(targetVelocity);
        updateOdometry();
    }

    // ---------------- CHASSIS (BOTH) ----------------

    /**
     * Command to drive both translational and rotational subsystems
     * using a dynamically supplied chassis velocity.
     */
    public Command getDriveVelocityCommand(Supplier<ChassisVelocity2d> velocitySupplier) {
        return new ParallelCommandGroup(
            getDriveTranslationalVelocity(() -> velocitySupplier.get().getTranslationalVelocity()),
            getDriveRotationalVelocity(() -> velocitySupplier.get().getRotationalVelocity())
        ).withRequirements(this);
    }

    /** Command to drive chassis using a fixed velocity */
    public Command getDriveVelocityCommand(ChassisVelocity2d velocity) {
        return getDriveVelocityCommand(() -> velocity);
    }

    /** Command to stop the chassis (zero velocity) */
    public Command getDriveVelocityCommandStop() {
        return getDriveVelocityCommand(new ChassisVelocity2d(new Translation2d(0.0, 0.0), 0.0));
    }

    // ---------------- TRANSLATIONAL ----------------

    /** Command to drive translational subsystem with a dynamic velocity */
    public Command getDriveTranslationalVelocity(Supplier<Translation2d> velocitySupplier) {
        return new RunCommand(
            () -> translationalSubsystem.setTranslationalVelocity(velocitySupplier.get())
        ).withRequirements(this.translationalSubsystem);
    }

    /** Command to drive translational subsystem with a fixed velocity */
    public Command getDriveTranslationalVelocity(Translation2d velocity) {
        return getDriveTranslationalVelocity(() -> velocity);
    }

    /** Command to stop translational motion */
    public Command getDriveTranslationalVelocityStop() {
        return getDriveTranslationalVelocity(new Translation2d(0.0, 0.0));
    }

    // ---------------- ROTATIONAL ----------------

    /** Command to drive rotational subsystem with a dynamic velocity */
    public Command getDriveRotationalVelocity(Supplier<Double> velocitySupplier) {
        return new RunCommand(
            () -> rotationalSubsystem.setRotationalVelocity(velocitySupplier.get())
        ).withRequirements(this.rotationalSubsystem);
    }

    /** Command to drive rotational subsystem with a fixed velocity */
    public Command getDriveRotationalVelocity(double velocity) {
        return getDriveRotationalVelocity(() -> velocity);
    }

    /** Command to stop rotational motion */
    public Command getDriveRotationalVelocityStop() {
        return getDriveRotationalVelocity(0.0);
    }

    // ---------------- DRIVE ----------------

    /** Command to drive to a dynamic pose using PID controllers */
    public Command getDriveToPoseCommand(Supplier<Pose2d> poseSupplier) {
        return new ParallelCommandGroup(
            getDriveToTranslationCommand(() -> poseSupplier.get().getTranslation()),
            getDriveToRotationCommand(() -> poseSupplier.get().getRotation().getRadians())
        );
    }

    /** Command to drive to a fixed pose using PID controllers */
    public Command getDriveToPoseCommand(Pose2d target) {
        return getDriveToPoseCommand(() -> target);
    }

    // ---------------- TRANSLATIONAL ----------------

    /** Command to drive translationally to a dynamic target using PID */
    public Command getDriveToTranslationCommand(Supplier<Translation2d> translationSupplier) {
        return getDriveTranslationalVelocity(
            () -> new Translation2d(
                xController.calculate(getRobotPose().getTranslation().getX(),
                                      translationSupplier.get().getX()),
                yController.calculate(getRobotPose().getTranslation().getY(),
                                      translationSupplier.get().getY())
            )
        );
    }

    /** Command to drive translationally to a fixed target using PID */
    public Command getDriveToTranslationCommand(Translation2d target) {
        return getDriveToTranslationCommand(() -> target);
    }

    // ---------------- ROTATIONAL ----------------

    /** Command to drive rotationally to a dynamic target using PID */
    public Command getDriveToRotationCommand(Supplier<Double> rotationRadiansSupplier) {
        return getDriveRotationalVelocity(
            () -> thetaController.calculate(getRobotPose().getRotation().getRadians(),
                                            rotationRadiansSupplier.get())
        );
    }

    /** Command to drive rotationally to a fixed target using PID */
    public Command getDriveToRotationCommand(double targetRadians) {
        return getDriveToRotationCommand(() -> targetRadians);
    }

    /** Set individual motor outputs based on chassis velocity */
    public void setMotors(ChassisVelocity2d velocity) {
        // Implementation depends on drivetrain (mecanum, omni, etc.)
    }

    /** Update robot pose (odometry) */
    public void updateOdometry() {
        // Implementation depends on sensors and wheel tracking
    }
}
