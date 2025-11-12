package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.HolonomicDriveConfig;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;
import org.firstinspires.ftc.teamcode.util.math.PIDFController;
import org.firstinspires.ftc.teamcode.util.math.RateLimiter;
import org.firstinspires.ftc.teamcode.util.math.RateLimiter2d;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

import java.util.function.Supplier;

/**
 * Abstract base class for any holonomic (e.g., mecanum or omniwheel) drivetrain.
 * Provides:
 * - Separate translational and rotational subsystems
 * - PIDF-controlled driving to positions
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
        private Translation2d translationalVelocity = new Translation2d(0.0, 0.0);

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

    /** PID controllers for X, Y translation and rotation */
    private PIDFController xController;
    private PIDFController yController;
    private PIDFController thetaController;

    /** Rate Limiters for limiting robot acceleration to prevent wheel slip */
    private RateLimiter2d translationalAccelerationLimiter;
    private RateLimiter rotationalAccelerationLimiter;

    private double maxTranslationalSpeed = 6.0;
    private double maxRotationalSpeed = 6.0;

    /** Constructor initializes subsystems and enables continuous input for rotation PID */
    public HolonomicDriveSubsystem(HolonomicDriveConfig config) {
        translationalSubsystem = new TranslationalSubsystem();
        rotationalSubsystem = new RotationalSubsystem();

        xController = new PIDFController(config.translationalPID);
        yController = new PIDFController(config.translationalPID);
        thetaController = new PIDFController(config.rotationalPID);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        maxTranslationalSpeed = config.maxTranslationalSpeed;
        maxRotationalSpeed = config.maxRotationalSpeed;

        translationalAccelerationLimiter = new RateLimiter2d(translationalSubsystem.getTranslationalVelocity(),config.maxTranslationalAccel,0.5);
        rotationalAccelerationLimiter = new RateLimiter(rotationalSubsystem.getRotationalVelocity(),config.maxRotationalAccel,0.5);
    }

    /**
     * Periodic update called by the scheduler.
     * <p>
     * This method:
     * <ol>
     *   <li>Reads desired translational and rotational velocities from their subsystems.</li>
     *   <li>Applies acceleration limiting (rate limiters).</li>
     *   <li>Caps the magnitudes of translational and rotational velocities to their maximum values.</li>
     *   <li>Builds a {@link ChassisVelocity2d} object from the limited values.</li>
     *   <li>Sends wheel speeds to motors.</li>
     *   <li>Updates odometry using encoder + IMU measurements.</li>
     * </ol>
     */
    @Override
    public void periodic() {

        // Get the desired velocities from subsystems and apply rate limiting
        Translation2d limitedTranslation =
            translationalAccelerationLimiter.update(translationalSubsystem.getTranslationalVelocity());
        double limitedRotation =
            rotationalAccelerationLimiter.update(rotationalSubsystem.getRotationalVelocity());

        // Cap the translational speed to the robot's max
        if (limitedTranslation.magnitude() > maxTranslationalSpeed) {
            limitedTranslation = limitedTranslation.withMagnitude(maxTranslationalSpeed);
        }

        // Cap the rotational speed to the robot's max
        limitedRotation = MathUtil.clamp(
            limitedRotation,
            -maxRotationalSpeed,
            maxRotationalSpeed
        );

        // Combine capped translation + rotation into chassis velocity
        ChassisVelocity2d targetVelocity = new ChassisVelocity2d(limitedTranslation, limitedRotation);

        // Send velocities to the drivetrain motors
        setMotors(targetVelocity);

        // Update robot odometry
        updateOdometry();
    }

    /** Set robot pose manually (e.g., for odometry reset) */
    public abstract void setRobotPose(Pose2d pose);

    /** Get the current robot pose */
    public abstract Pose2d getRobotPose();

    /** Set individual motor outputs based on chassis velocity */
    public abstract void setMotors(ChassisVelocity2d velocity);

    /** Update robot pose (odometry) */
    public abstract void updateOdometry();

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
        return new InstantCommand(
                () -> {
                    translationalSubsystem.setTranslationalVelocity(Translation2d.ZERO);
                    rotationalSubsystem.setRotationalVelocity(0.0);
                }
        );
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

    /** Command to drive to a dynamic pose using PIDF controllers */
    public Command getDriveToPoseCommand(Supplier<Pose2d> poseSupplier) {
        return new ParallelCommandGroup(
            getDriveToTranslationCommand(() -> poseSupplier.get().getTranslation()),
            getDriveToRotationCommand(() -> poseSupplier.get().getRotation().getRadians())
        );
    }

    /** Command to drive to a fixed pose using PIDF controllers */
    public Command getDriveToPoseCommand(Pose2d target) {
        return getDriveToPoseCommand(() -> target);
    }

    // ---------------- TRANSLATIONAL ----------------

    /** Command to drive translationally to a dynamic target using PIDF */
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

    /** Command to drive translationally to a fixed target using PIDF */
    public Command getDriveToTranslationCommand(Translation2d target) {
        return getDriveToTranslationCommand(() -> target);
    }

    // ---------------- ROTATIONAL ----------------

    /** Command to drive rotationally to a dynamic target using PIDF */
    public Command getDriveToRotationCommand(Supplier<Double> rotationRadiansSupplier) {
        return getDriveRotationalVelocity(
            () -> thetaController.calculate(getRobotPose().getRotation().getRadians(),
                                            rotationRadiansSupplier.get())
        );
    }

    /** Command to drive rotationally to a fixed target using PIDF */
    public Command getDriveToRotationCommand(double targetRadians) {
        return getDriveToRotationCommand(() -> targetRadians);
    }

}
