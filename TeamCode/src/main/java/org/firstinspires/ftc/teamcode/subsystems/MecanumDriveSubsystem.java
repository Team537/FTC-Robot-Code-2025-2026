package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.HolonomicDriveConfig;
import org.firstinspires.ftc.teamcode.util.mecanum.MecanumDriveConfig;
import org.firstinspires.ftc.teamcode.util.mecanum.MecanumPoseEstimator;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;

import java.util.Arrays;

/**
 * Mecanum drivetrain subsystem for FTC.
 *
 * Responsibilities:
 * - Manage 4 mecanum wheels (FL, FR, BL, BR)
 * - Drive the robot with field-relative or robot-relative velocities
 * - Convert chassis velocity → wheel velocities
 * - Update robot pose using mecanum odometry + IMU heading
 */
public class MecanumDriveSubsystem extends HolonomicDriveSubsystem {

    private MecanumDriveConfig config;

    // ---------------- MOTORS ----------------
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    // ---------------- SENSORS ----------------
    private IMU imu;

    // ---------------- ODOMETRY ----------------
    private MecanumPoseEstimator mecanumPoseEstimator;

    private double inchesPerTick;

    /**
     * Constructor: initializes motors, IMU, and odometry
     */
    public MecanumDriveSubsystem(HardwareMap hardwareMap, MecanumDriveConfig config) {
        super(
            new HolonomicDriveConfig()
                .setTranslationalPID(config.translationalPID)
                .setRotationalPID(config.rotationalPID)
                .setMaxTranslationalSpeed(config.maxTranslationalSpeed)
                .setMaxRotationalSpeed(config.maxRotationalSpeed)
                .setMaxTranslationalAccel(config.maxTranslationalAccel)
                .setMaxRotationalAccel(config.maxRotationalAccel)
        );

        this.config = config;

        // Initialize motors from hardware map
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, config.frontLeftMotorName);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, config.frontRightMotorName);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, config.backLeftMotorName);
        backRightMotor = hardwareMap.get(DcMotorEx.class, config.backRightMotorName);

        // Set motor directions according to configuration constants
        frontLeftMotor.setDirection(config.frontLeftMotorDirection);
        frontRightMotor.setDirection(config.frontRightMotorDirection);
        backLeftMotor.setDirection(config.backLeftMotorDirection);
        backRightMotor.setDirection(config.backRightMotorDirection);

        frontLeftMotor.setVelocityPIDFCoefficients(config.motorVelocityPID.kP,config.motorVelocityPID.kI,config.motorVelocityPID.kD,config.motorVelocityPID.kF.get());
        frontRightMotor.setVelocityPIDFCoefficients(config.motorVelocityPID.kP,config.motorVelocityPID.kI,config.motorVelocityPID.kD,config.motorVelocityPID.kF.get());
        backLeftMotor.setVelocityPIDFCoefficients(config.motorVelocityPID.kP,config.motorVelocityPID.kI,config.motorVelocityPID.kD,config.motorVelocityPID.kF.get());
        backRightMotor.setVelocityPIDFCoefficients(config.motorVelocityPID.kP,config.motorVelocityPID.kI,config.motorVelocityPID.kD,config.motorVelocityPID.kF.get());

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, config.imuName);

        // Force reading to initialize
        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Setup odometry
        mecanumPoseEstimator = new MecanumPoseEstimator(config.kinematics, Pose2d.ZERO);
        mecanumPoseEstimator.resetPose(Pose2d.ZERO, getIMUHeading());

        // Calculate conversion: ticks/sec → inches/sec
        double wheelCircumference = config.wheelCircumference;
        this.inchesPerTick = wheelCircumference / config.ticksPerRevolution;

    }

    /**
     * Returns the robot heading as a Rotation2d using the IMU yaw angle.
     */
    public Rotation2d getIMUHeading() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    /**
     * Periodic update method called by scheduler.
     * - Calls superclass periodic for drive control
     * - Sends robot pose telemetry
     */
    public void periodic() {
        super.periodic();
        TelemetryManager.put("Position", getRobotPose().toString());
    }

    // ---------------- DRIVE ----------------

    /**
     * Convert chassis velocity → wheel velocities and send to motors.
     * - Uses proportional scaling to prevent exceeding max wheel speed
     * - Converts inches/sec → ticks/sec for FTC motors
     */
    @Override
    public void setMotors(ChassisVelocity2d chassisVelocity) {

        // Set motor directions according to configuration constants
        frontLeftMotor.setDirection(config.frontLeftMotorDirection);
        frontRightMotor.setDirection(config.frontRightMotorDirection);
        backLeftMotor.setDirection(config.backLeftMotorDirection);
        backRightMotor.setDirection(config.backRightMotorDirection);

        // Convert chassis velocity to individual wheel speeds
        // Convert to field relative first
        double[] wheelSpeeds = config.kinematics.toWheelSpeeds(chassisVelocity.toRobotRelative(getRobotPose().getRotation()));

        // Proportional scaling: prevent wheel speed from exceeding max
        double max = Arrays.stream(wheelSpeeds).map(Math::abs).max().orElse(0.0);
        if (max > config.maxWheelSpeed) {
            double scale = config.maxWheelSpeed / max;
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] *= scale;
            }
        }

        TelemetryManager.put("Reported Speed", config.kinematics.fromWheelSpeeds(wheelSpeeds).toString());

        // Convert wheel speed from inches/sec → encoder ticks/sec
        double[] wheelSpeedsTicksPerSec = new double[4];
        for (int i = 0; i < 4; i++) {
            wheelSpeedsTicksPerSec[i] = wheelSpeeds[i] / inchesPerTick;
        }

        // Send telemetry for debugging
        TelemetryManager.put("Wheel Speeds FL (tics/s)", wheelSpeedsTicksPerSec[0]);
        TelemetryManager.put("Wheel Speeds FR (tics/s)", wheelSpeedsTicksPerSec[1]);
        TelemetryManager.put("Wheel Speeds BL (tics/s)", wheelSpeedsTicksPerSec[2]);
        TelemetryManager.put("Wheel Speeds BR (tics/s)", wheelSpeedsTicksPerSec[3]);

        TelemetryManager.put("Measured Wheel Speeds FL (tics/s)", frontLeftMotor.getVelocity());
        TelemetryManager.put("Measured Wheel Speeds FR (tics/s)", frontRightMotor.getVelocity());
        TelemetryManager.put("Measured Wheel Speeds BL (tics/s)", backLeftMotor.getVelocity());
        TelemetryManager.put("Measured Wheel Speeds BR (tics/s)", backRightMotor.getVelocity());

        TelemetryManager.put("Motor Direction FL", frontLeftMotor.getDirection());
        TelemetryManager.put("Motor Direction FR", frontRightMotor.getDirection());
        TelemetryManager.put("Motor Direction BL", backLeftMotor.getDirection());
        TelemetryManager.put("Motor Direction BR", backRightMotor.getDirection());

        // Apply velocities to motors
        frontLeftMotor.setVelocity(wheelSpeedsTicksPerSec[0]);
        frontRightMotor.setVelocity(wheelSpeedsTicksPerSec[1]);
        backLeftMotor.setVelocity(wheelSpeedsTicksPerSec[2]);
        backRightMotor.setVelocity(wheelSpeedsTicksPerSec[3]);
    }

    // ---------------- ODOMETRY ----------------

    /**
     * Update robot pose using mecanum wheel encoders + IMU heading.
     * Steps:
     * 1. Read current motor positions
     * 2. Convert ticks → inches
     * 3. Feed distances + heading into MecanumPoseEstimator
     * 4. Update HolonomicDriveSubsystem robotPose
     */
    @Override
    public void updateOdometry() {

        // Get current motor encoder positions
        int[] currentPositions = new int[] {
            frontLeftMotor.getCurrentPosition(),
            frontRightMotor.getCurrentPosition(),
            backLeftMotor.getCurrentPosition(),
            backRightMotor.getCurrentPosition()
        };

        double[] currentPositionInches = new double[4];
        for (int i = 0; i < 4; i++) {
            currentPositionInches[i] = currentPositions[i] * inchesPerTick;
        }

        // Update pose estimator with distances + IMU heading
        mecanumPoseEstimator.update(currentPositionInches, getIMUHeading());

    }

    /** Get the current robot pose */
    public Pose2d getRobotPose() {
        return mecanumPoseEstimator.getEstimatedPose();
    }

    /**
     * Resets the robot pose
     * @param pose The pose to reset to
     */
    public void setRobotPose(Pose2d pose) {
        mecanumPoseEstimator.resetPose(pose,getIMUHeading());
    }
}
