package org.firstinspires.ftc.teamcode.util.mecanum;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.util.math.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.mecanum.MecanumDriveKinematics;

public class MecanumDriveConfig {

    // --- Kinematics ---
    public MecanumDriveKinematics kinematics;
    public double wheelCircumference = 0.0;
    public double ticksPerRevolution = 0.0;

    // --- Controllers ---
    public PIDFCoefficients translationalPID = new PIDFCoefficients();
    public PIDFCoefficients rotationalPID = new PIDFCoefficients();
    public PIDFCoefficients motorVelocityPID = new PIDFCoefficients();

    // --- Motors ---
    public String frontLeftMotorName;
    public String frontRightMotorName;
    public String backLeftMotorName;
    public String backRightMotorName;

    public DcMotorSimple.Direction frontLeftMotorDirection;
    public DcMotorSimple.Direction frontRightMotorDirection;
    public DcMotorSimple.Direction backLeftMotorDirection;
    public DcMotorSimple.Direction backRightMotorDirection;

    // --- IMU ---
    public String imuName;
    public boolean imuInverted = false;

    // --- Physical constants ---
    public double maxWheelSpeed = 0.0;
    public double maxTranslationalSpeed = 0.0;
    public double maxRotationalSpeed = 0.0;
    public double maxTranslationalAccel = 0.0;
    public double maxRotationalAccel = 0.0;

    // --- Fluent setters ---
    public MecanumDriveConfig setKinematics(MecanumDriveKinematics k) {
        this.kinematics = k;
        return this;
    }

    public MecanumDriveConfig setWheelCircumference(double c) {
        this.wheelCircumference = c;
        return this;
    }

    public MecanumDriveConfig setTicksPerRevolution(double tpr) {
        this.ticksPerRevolution = tpr;
        return this;
    }

    public MecanumDriveConfig setTranslationalPID(PIDFCoefficients pid) {
        this.translationalPID = pid;
        return this;
    }

    public MecanumDriveConfig setRotationalPID(PIDFCoefficients pid) {
        this.rotationalPID = pid;
        return this;
    }

    public MecanumDriveConfig setMotorVelocityPID(PIDFCoefficients pid) {
        this.motorVelocityPID = pid;
        return this;
    }

    public MecanumDriveConfig setFrontLeftMotor(String name, DcMotorSimple.Direction dir) {
        this.frontLeftMotorName = name;
        this.frontLeftMotorDirection = dir;
        return this;
    }

    public MecanumDriveConfig setFrontRightMotor(String name, DcMotorSimple.Direction dir) {
        this.frontRightMotorName = name;
        this.frontRightMotorDirection = dir;
        return this;
    }

    public MecanumDriveConfig setBackLeftMotor(String name, DcMotorSimple.Direction dir) {
        this.backLeftMotorName = name;
        this.backLeftMotorDirection = dir;
        return this;
    }

    public MecanumDriveConfig setBackRightMotor(String name, DcMotorSimple.Direction dir) {
        this.backRightMotorName = name;
        this.backRightMotorDirection = dir;
        return this;
    }

    public MecanumDriveConfig setIMU(String name, boolean inverted) {
        this.imuName = name;
        this.imuInverted = inverted;
        return this;
    }

    public MecanumDriveConfig setMaxWheelSpeed(double s) {
        this.maxWheelSpeed = s;
        return this;
    }

    public MecanumDriveConfig setMaxTranslationalSpeed(double s) {
        this.maxTranslationalSpeed = s;
        return this;
    }

    public MecanumDriveConfig setMaxRotationalSpeed(double s) {
        this.maxRotationalSpeed = s;
        return this;
    }

    public MecanumDriveConfig setMaxTranslationalAccel(double a) {
        this.maxTranslationalAccel = a;
        return this;
    }

    public MecanumDriveConfig setMaxRotationalAccel(double a) {
        this.maxRotationalAccel = a;
        return this;
    }
}
