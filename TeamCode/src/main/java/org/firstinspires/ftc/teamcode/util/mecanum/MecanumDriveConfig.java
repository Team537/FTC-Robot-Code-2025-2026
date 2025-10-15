package org.firstinspires.ftc.teamcode.util.mecanum;

import org.firstinspires.ftc.teamcode.util.mecanum.MecanumDriveKinematics;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDriveConfig {

    // Motor info
    public final String frontLeftMotorName;
    public final String frontRightMotorName;
    public final String backLeftMotorName;
    public final String backRightMotorName;

    public final DcMotorEx.Direction frontLeftDirection;
    public final DcMotorEx.Direction frontRightDirection;
    public final DcMotorEx.Direction backLeftDirection;
    public final DcMotorEx.Direction backRightDirection;

    // Wheel / encoder info
    public final double wheelRadius;        // inches
    public final double gearRatio;          // motor -> wheel
    public final double encoderTicksPerRev; // motor encoder ticks per revolution

    // Robot dimensions for kinematics
    public final double wheelBase;  // front-back distance (inches)
    public final double trackWidth; // left-right distance (inches)

    // Kinematics object
    public final MecanumDriveKinematics kinematics;

    // IMU orientation / mounting
    public final String imuName;
    public final RevHubOrientationOnRobot hubOrientation; // radians to adjust if IMU is mounted rotated

    // Optional: max speeds & acceleration limits
    public final double maxTranslationalSpeed; // inches/sec
    public final double maxRotationalSpeed;    // radians/sec

    public MecanumDriveConfig(
        String flName, String frName, String blName, String brName,
        DcMotorEx.Direction flDir, DcMotorEx.Direction frDir,
        DcMotorEx.Direction blDir, DcMotorEx.Direction brDir,
        double wheelRadius, double gearRatio, double encoderTicksPerRev,
        double wheelBase, double trackWidth,
        String imuName,
        RevHubOrientationOnRobot hubOrientation,
        double maxTranslationalSpeed, double maxRotationalSpeed
    ) {
        this.frontLeftMotorName = flName;
        this.frontRightMotorName = frName;
        this.backLeftMotorName = blName;
        this.backRightMotorName = brName;

        this.frontLeftDirection = flDir;
        this.frontRightDirection = frDir;
        this.backLeftDirection = blDir;
        this.backRightDirection = brDir;

        this.wheelRadius = wheelRadius;
        this.gearRatio = gearRatio;
        this.encoderTicksPerRev = encoderTicksPerRev;

        this.wheelBase = wheelBase;
        this.trackWidth = trackWidth;

        this.kinematics = new MecanumDriveKinematics(wheelBase, trackWidth);

        this.imuName = imuName;
        this.hubOrientation = hubOrientation;

        this.maxTranslationalSpeed = maxTranslationalSpeed;
        this.maxRotationalSpeed = maxRotationalSpeed;
    }
}
