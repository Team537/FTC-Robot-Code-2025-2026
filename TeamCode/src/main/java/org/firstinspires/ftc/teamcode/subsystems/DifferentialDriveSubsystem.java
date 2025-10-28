package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.DifferentialDriveConfig;
import org.firstinspires.ftc.teamcode.util.DifferentialPoseEstimator;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.DifferentialDriveKinematics;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.util.math.PIDFController;
import org.firstinspires.ftc.teamcode.util.math.RateLimiter;

import java.util.function.Supplier;

public class DifferentialDriveSubsystem extends Subsystem {

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private IMU imu;


    private final DifferentialDriveKinematics kinematics;
    private final DifferentialPoseEstimator differentialPoseEstimator;

    // Conversion factor: motor encoder ticks → inches/sec
    private final double inchesPerTick;

    private ChassisVelocity2d targetVelocity = new ChassisVelocity2d(Translation2d.ZERO,0.0);

    private PIDFController translationalController;
    private PIDFController rotationalController;

    private RateLimiter translationalAccelerationLimiter;
    private RateLimiter rotationalAccelerationLimiter;


    private boolean imuInverted;

    public DifferentialDriveSubsystem(HardwareMap hardwareMap, DifferentialDriveConfig config) {
        leftMotor = hardwareMap.get(DcMotorEx.class, config.leftMotorName);
        rightMotor = hardwareMap.get(DcMotorEx.class, config.rightMotorName);

        leftMotor.setDirection(config.leftMotorDirection);
        rightMotor.setDirection(config.rightMotorDirection);

        translationalController = new PIDFController(config.translationalPID);
        rotationalController = new PIDFController(config.rotationalPID);

        translationalAccelerationLimiter = new RateLimiter(0.0,config.maxTranslationalAccel,0.5);
        rotationalAccelerationLimiter = new RateLimiter(0.0,config.maxRotationalAccel,0.5);

        leftMotor.setVelocityPIDFCoefficients(config.motorVelocityPID.kP,config.motorVelocityPID.kI,config.motorVelocityPID.kD,config.motorVelocityPID.kF.get());
        rightMotor.setVelocityPIDFCoefficients(config.motorVelocityPID.kP,config.motorVelocityPID.kI,config.motorVelocityPID.kD,config.motorVelocityPID.kF.get());

        this.kinematics = config.kinematics;
        this.differentialPoseEstimator = new DifferentialPoseEstimator(kinematics,Pose2d.ZERO);

        // Calculate conversion: ticks/sec → inches/sec
        double wheelCircumference = config.wheelCircumference;
        this.inchesPerTick = wheelCircumference / config.ticksPerRevolution;

        rotationalController.enableContinuousInput(-Math.PI,Math.PI);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, config.imuName);

        imuInverted = config.imuInverted;

        // Force reading to initialize
        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Reset odometry to origin
        differentialPoseEstimator.resetPose(Pose2d.ZERO, getIMUHeading());

    }

    /**
     * Returns the robot heading as a Rotation2d using the IMU yaw angle.
     */
    public Rotation2d getIMUHeading() {
        return new Rotation2d( (imuInverted ? -1.0 : 1.0) * imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    @Override
    public void periodic() {

        ChassisVelocity2d velocity =
            new ChassisVelocity2d(
                new Translation2d(
                    translationalAccelerationLimiter.update(targetVelocity.getTranslationalVelocity().getX()),
                    0.0
                ),
                rotationalAccelerationLimiter.update(targetVelocity.getRotationalVelocity())
            );

        setMotors(velocity);
        updateOdometry();
    }

    /**
     * Sets motor targets based on desired linear and angular velocity.
     *
     * @param velocity The velocity in inches per second and radians per second. The y-value isn't used.
     */
    public void setMotors(ChassisVelocity2d velocity) {
        double[] wheelSpeeds = kinematics.toWheelSpeeds(velocity);

        TelemetryManager.put("Left Wheel Target Speed", wheelSpeeds[0]);
        TelemetryManager.put("Right Wheel Target Speed", wheelSpeeds[1]);

        // Convert inches/sec → ticks/sec
        double leftTicksPerSec = wheelSpeeds[0] / inchesPerTick;
        double rightTicksPerSec = wheelSpeeds[1] / inchesPerTick;

        leftMotor.setVelocity(leftTicksPerSec);
        rightMotor.setVelocity(rightTicksPerSec);
    }

    /**
     * Update robot pose using differential wheel encoders + IMU heading.
     * Steps:
     * 1. Read current motor positions
     * 2. Convert ticks → inches
     * 3. Feed distances + heading into DifferentialPoseEstimator
     * 4. Update DifferentialDriveSubsystem robotPose
     */
    public void updateOdometry() {

        // Get current motor encoder positions
        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();

        // Convert ticks → inches
        double leftInches = leftPosition * inchesPerTick;
        double rightInches = rightPosition * inchesPerTick;

        // Update pose estimator with wheel distances + IMU heading
        differentialPoseEstimator.update(new double[]{leftInches, rightInches}, getIMUHeading());

    }

    /** Command factory for velocity control. */
    public Command getDriveVelocityCommand(
        Supplier<ChassisVelocity2d> velocitySupplier
    ) {
        return new RunCommand(
            () -> targetVelocity = velocitySupplier.get()
        ).withRequirements(this);
    }

    /** Command factory for velocity control (static/fixed target). */
    public Command getDriveVelocityCommand(ChassisVelocity2d velocity) {
        // Wrap the fixed velocity in a supplier
        return getDriveVelocityCommand(() -> velocity);
    }

    /** Command factory to stop the drivetrain (zero chassis velocity). */
    public Command getVelocityCommandStop() {
        return getDriveVelocityCommand(new ChassisVelocity2d(new Translation2d(0.0, 0.0), 0.0));
    }

    /**
     * Returns a command that drives the robot to a target pose.
     * Works for a differential drivetrain (no strafing).
     */
    public Command getDriveToPoseCommand(Supplier<Pose2d> targetPoseSupplier) {
        return new RunCommand(() -> {
            // Current pose
            Pose2d currentPose = getRobotPose();
            Pose2d targetPose = targetPoseSupplier.get();

            // Vector from robot to target in field coordinates
            Translation2d delta = targetPose.getTranslation().minus(currentPose.getTranslation());

            // Heading to target (angle from robot position to target)
            double targetHeadingToPoint = Math.atan2(delta.getY(), delta.getX());

            // Forward distance along robot's current heading
            double headingErrorToPoint = targetHeadingToPoint - currentPose.getRotation().getRadians();
            double forwardDistance = delta.magnitude() * Math.cos(headingErrorToPoint);

            // Translational velocity (PID)
            double translationalSpeed = translationalController.calculate(forwardDistance, 0.0);

            // Rotational velocity (PID) to target orientation
            double headingError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();
            double rotationalSpeed = rotationalController.calculate(headingError, 0.0);

            // Send velocities to drivetrain
            targetVelocity = new ChassisVelocity2d(new Translation2d(translationalSpeed,0.0),rotationalSpeed);
        }).withRequirements(this);
    }

    /**
     * Overload: static target pose
     */
    public Command getDriveToPoseCommand(Pose2d targetPose) {
        return getDriveToPoseCommand(() -> targetPose);
    }

    /** Stops motors. */
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /** Get the current robot pose */
    public Pose2d getRobotPose() {
        return differentialPoseEstimator.getEstimatedPose();
    }

    /**
     * Resets the robot's pose
     * @param pose the pose to reset to
     */
    public void setRobotPose(Pose2d pose) {
        differentialPoseEstimator.resetPose(pose,getIMUHeading());
    }

}
