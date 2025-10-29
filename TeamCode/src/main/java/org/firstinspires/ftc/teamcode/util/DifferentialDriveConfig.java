package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.math.PIDFCoefficients;

public class DifferentialDriveConfig {

    // --- Kinematics ---
    public DifferentialDriveKinematics kinematics;
    public double wheelCircumference = 0.0;
    public double ticksPerRevolution = 0.0;

    // --- Controllers ---
    public PIDFCoefficients translationalPID = new PIDFCoefficients();
    public PIDFCoefficients rotationalPID = new PIDFCoefficients();
    public PIDFCoefficients motorVelocityPID = new PIDFCoefficients();

    // --- Motors ---
    public String leftMotorName;
    public String rightMotorName;
    public DcMotorSimple.Direction leftMotorDirection;
    public DcMotorSimple.Direction rightMotorDirection;

    // --- IMU ---
    public String imuName;
    public boolean imuInverted = false;

    // --- Physical constants ---
    public double maxTranslationalSpeed = 0.0;
    public double maxRotationalSpeed = 0.0;
    public double maxTranslationalAccel = 0.0;
    public double maxRotationalAccel = 0.0;

    // Fluent setters for convenience
    public DifferentialDriveConfig setKinematics(DifferentialDriveKinematics k) { this.kinematics = k; return this; }
    public DifferentialDriveConfig setWheelCircumference(double c) { this.wheelCircumference = c; return this; }
    public DifferentialDriveConfig setTicksPerRevolution(double tpr) { this.ticksPerRevolution = tpr; return this; }

    public DifferentialDriveConfig setTranslationalPID(PIDFCoefficients pid) { this.translationalPID = pid; return this; }
    public DifferentialDriveConfig setRotationalPID(PIDFCoefficients pid) { this.rotationalPID = pid; return this; }
    public DifferentialDriveConfig setMotorVelocityPID(PIDFCoefficients pid) { this.motorVelocityPID = pid; return this; }

    public DifferentialDriveConfig setLeftMotorName(String name) { this.leftMotorName = name; return this; }
    public DifferentialDriveConfig setRightMotorName(String name) { this.rightMotorName = name; return this; }
    public DifferentialDriveConfig setLeftMotorDirection(DcMotorSimple.Direction dir) { this.leftMotorDirection = dir; return this; }
    public DifferentialDriveConfig setRightMotorDirection(DcMotorSimple.Direction dir) { this.rightMotorDirection = dir; return this; }

    public DifferentialDriveConfig setIMU(String name, boolean inverted) { this.imuName = name; this.imuInverted = inverted; return this; }

    public DifferentialDriveConfig setMaxTranslationalSpeed(double s) { this.maxTranslationalSpeed = s; return this; }
    public DifferentialDriveConfig setMaxRotationalSpeed(double s) { this.maxRotationalSpeed = s; return this; }
    public DifferentialDriveConfig setMaxTranslationalAccel(double a) { this.maxTranslationalAccel = a; return this; }
    public DifferentialDriveConfig setMaxRotationalAccel(double a) { this.maxRotationalAccel = a; return this; }
}
