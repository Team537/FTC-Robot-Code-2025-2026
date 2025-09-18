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

public abstract class HolonomicDriveSubsystem extends Subsystem {

    private TranslationalSubsystem translationalSubsystem;
    private RotationalSubsystem rotationalSubsystem;

    private class TranslationalSubsystem extends Subsystem {

        private Translation2d translationalVelocity = new Translation2d(0.0,0.0);

        public void setTranslationalVelocity(Translation2d velocity) {
            this.translationalVelocity = velocity;
        }

        public Translation2d getTranslationalVelocity() {
            return translationalVelocity;
        }

    }

    private class RotationalSubsystem extends Subsystem {

        private double rotationalVelocity = 0.0;

        public void setRotationalVelocity(double velocity) {
            this.rotationalVelocity= velocity;
        }

        public double getRotationalVelocity() {
            return rotationalVelocity;
        }

    }

    private Pose2d robotPose = new Pose2d(new Translation2d(0.0,0.0),new Rotation2d(0.0));

    public void setRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    private PIDController xController = new PIDController(0.0,0.0,0.0);
    private PIDController yController = new PIDController(0.0,0.0,0.0);
    private PIDController thetaController = new PIDController(0.0,0.0,0.0);

    public HolonomicDriveSubsystem() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        translationalSubsystem = new TranslationalSubsystem();
        rotationalSubsystem = new RotationalSubsystem();
    }

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
     * Returns a command that drives both translational and rotational subsystems
     * using a supplied ChassisVelocity2d.
     *
     * @param velocitySupplier supplier of chassis velocity
     * @return command to drive the robot
     */
    public Command getDriveVelocityCommand(Supplier<ChassisVelocity2d> velocitySupplier) {
        return new ParallelCommandGroup(
            getDriveTranslationalVelocity(() -> velocitySupplier.get().getTranslationalVelocity()),
            getDriveRotationalVelocity(() -> velocitySupplier.get().getRotationalVelocity())
        );
    }

    /**
     * Drive both subsystems using a fixed ChassisVelocity2d.
     */
    public Command getDriveVelocityCommand(ChassisVelocity2d velocity) {
        return getDriveVelocityCommand(() -> velocity);
    }

    /**
     * Stop both subsystems (zero chassis velocity).
     */
    public Command getDriveVelocityCommandStop() {
        return getDriveVelocityCommand(new ChassisVelocity2d(new Translation2d(0.0, 0.0), 0.0));
    }

    // ---------------- TRANSLATIONAL ----------------

    /**
     * Drive translational subsystem using a supplied velocity.
     */
    public Command getDriveTranslationalVelocity(Supplier<Translation2d> velocitySupplier) {
        return new RunCommand(
            () -> translationalSubsystem.setTranslationalVelocity(velocitySupplier.get())
        );
    }

    /**
     * Drive translational subsystem using a fixed velocity.
     */
    public Command getDriveTranslationalVelocity(Translation2d velocity) {
        return getDriveTranslationalVelocity(() -> velocity);
    }

    /**
     * Stop the translational subsystem (zero velocity).
     */
    public Command getDriveTranslationalVelocityStop() {
        return getDriveTranslationalVelocity(new Translation2d(0.0, 0.0));
    }

    // ---------------- ROTATIONAL ----------------

    /**
     * Drive rotational subsystem using a supplied velocity.
     */
    public Command getDriveRotationalVelocity(Supplier<Double> velocitySupplier) {
        return new RunCommand(
            () -> rotationalSubsystem.setRotationalVelocity(velocitySupplier.get())
        );
    }

    /**
     * Drive rotational subsystem using a fixed velocity.
     */
    public Command getDriveRotationalVelocity(double velocity) {
        return getDriveRotationalVelocity(() -> velocity);
    }

    /**
     * Stop the rotational subsystem (zero velocity).
     */
    public Command getDriveRotationalVelocityStop() {
        return getDriveRotationalVelocity(0.0);
    }

    // ---------------- DRIVE ----------------

    /**
     * Drive to a full pose using PID controllers (translational + rotational).
     * Dynamic version: target supplied by a Supplier<Pose2d>.
     */
    public Command getDriveToPoseCommand(Supplier<Pose2d> poseSupplier) {
        return new ParallelCommandGroup(
            getDriveToTranslationCommand(() -> poseSupplier.get().getTranslation()),
            getDriveToRotationCommand(() -> poseSupplier.get().getRotation().getRadians())
        );
    }

    /**
     * Drive to a full pose using PID controllers (translational + rotational).
     * Static version: target is a fixed Pose2d.
     */
    public Command getDriveToPoseCommand(Pose2d target) {
        return getDriveToPoseCommand(() -> target);
    }

// ---------------- TRANSLATIONAL ----------------

    /**
     * Drive translationally to a target using PID controllers.
     * Dynamic version: target supplied by a Supplier<Translation2d>.
     */
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

    /**
     * Drive translationally to a fixed target using PID controllers.
     * Static version: target is a fixed Translation2d.
     */
    public Command getDriveToTranslationCommand(Translation2d target) {
        return getDriveToTranslationCommand(() -> target);
    }

// ---------------- ROTATIONAL ----------------

    /**
     * Drive rotationally to a target using PID controller.
     * Dynamic version: target supplied by a Supplier<Double> (radians).
     */
    public Command getDriveToRotationCommand(Supplier<Double> rotationRadiansSupplier) {
        return getDriveRotationalVelocity(
            () -> thetaController.calculate(getRobotPose().getRotation().getRadians(),
                                            rotationRadiansSupplier.get())
        );
    }

    /**
     * Drive rotationally to a fixed target using PID controller.
     * Static version: target is a fixed rotation in radians.
     */
    public Command getDriveToRotationCommand(double targetRadians) {
        return getDriveToRotationCommand(() -> targetRadians);
    }

    public void setMotors(ChassisVelocity2d velocity) {

    }

    public void updateOdometry() {

    }

}
