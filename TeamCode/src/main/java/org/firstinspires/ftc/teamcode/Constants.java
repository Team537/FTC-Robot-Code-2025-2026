package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.DifferentialDriveConfig;
import org.firstinspires.ftc.teamcode.util.DifferentialDriveKinematics;
import org.firstinspires.ftc.teamcode.util.math.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.mecanum.MecanumDriveConfig;
import org.firstinspires.ftc.teamcode.util.mecanum.MecanumDriveKinematics;
import org.firstinspires.ftc.teamcode.util.ExtraShooterConfig;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.util.ShooterConfig;

public class Constants {

    public enum OpModes {
        TELE_OP,
        AUTO,
        PRE_MATCH_INIT
    }

    public static class Drive {

        public static final String FRONT_LEFT_MOTOR_NAME = "front left";
        public static final String FRONT_RIGHT_MOTOR_NAME = "front right";
        public static final String BACK_LEFT_MOTOR_NAME = "back left";
        public static final String BACK_RIGHT_MOTOR_NAME = "back right";

        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final String IMU_NAME = "imu";

        public static final double WHEEL_BASE = 14.25;
        public static final double TRACK_WIDTH = 17.50;
        public static final MecanumDriveKinematics KINEMATICS = new MecanumDriveKinematics(WHEEL_BASE, TRACK_WIDTH);
        public static final double WHEEL_CIRCUMFERENCE = (Math.PI) * 4.0;

        public static final double GEAR_RATIO = 19.2;
        public static final double ENCODER_PULSE_PER_REVOLUTION = 28.0;
        public static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_PULSE_PER_REVOLUTION;

        public static final double MAX_WHEEL_SPEED = 60.0;
        public static final double MAX_TRANSLATIONAL_SPEED = MAX_WHEEL_SPEED / Math.sqrt(2.0); // inches per second
        public static final double MAX_ROTATIONAL_SPEED = (MAX_TRANSLATIONAL_SPEED) / (WHEEL_BASE / 2.0 + TRACK_WIDTH / 2.0);


        public static final double MAX_TRANSLATIONAL_ACCELERATION = 200.0;
        public static final double MAX_ROTATIONAL_ACCELERATION = 20.0;

        public static final MecanumDriveConfig MECANUM_DRIVE_CONFIG = new MecanumDriveConfig()
            .setFrontLeftMotor(FRONT_LEFT_MOTOR_NAME,FRONT_LEFT_MOTOR_DIRECTION)
            .setFrontRightMotor(FRONT_RIGHT_MOTOR_NAME,FRONT_RIGHT_MOTOR_DIRECTION)
            .setBackLeftMotor(BACK_LEFT_MOTOR_NAME,BACK_LEFT_MOTOR_DIRECTION)
            .setBackRightMotor(BACK_RIGHT_MOTOR_NAME,BACK_RIGHT_MOTOR_DIRECTION)
            .setIMU(IMU_NAME,true)
            .setKinematics(KINEMATICS)
            .setMaxWheelSpeed(MAX_WHEEL_SPEED)
            .setMaxTranslationalSpeed(MAX_TRANSLATIONAL_SPEED)
            .setMaxRotationalSpeed(MAX_ROTATIONAL_SPEED)
            .setMaxTranslationalAccel(MAX_TRANSLATIONAL_ACCELERATION)
            .setMaxRotationalAccel(MAX_ROTATIONAL_ACCELERATION)
            .setWheelCircumference(WHEEL_CIRCUMFERENCE)
            .setTicksPerRevolution(TICKS_PER_REVOLUTION)
            .setMotorVelocityPID(new PIDFCoefficients(1.0,0.0,0.0,13.0))
            .setTranslationalPID(new PIDFCoefficients(1.0,0.0,0.0,0.0))
            .setRotationalPID(new PIDFCoefficients(1.0,0.0,0.0,0.0));

    }

    public static class Assemblys {
        public static final String hopperMotor = "hopper motor";

        public static final String intakeMotor = "intake motor";

        public static final String shooterMotorLeft = "shooter motor left";

        public static final String shooterMotorRight = "shooter motor right";
    }

    public static class DifferentialDrive {

        public static final String LEFT_MOTOR_NAME = "leftMotor";
        public static final String RIGHT_MOTOR_NAME = "rightMotor";

        public static final DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

        public static final double TRACK_WIDTH = 10;
        public static final Translation2d PIVOT_OFFSET = new Translation2d(
            0.0,
            0.0
        );
        public static final double WHEEL_DIAMETER_INCHES = 3;
        public static final double TICKS_PER_REV = 7;
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
            TRACK_WIDTH,
            PIVOT_OFFSET
        );

        public static final double WHEEL_CIRCUMFERENCE = (Math.PI) * 4.0;

        public static final double GEAR_RATIO = 19.2;
        public static final double ENCODER_PULSE_PER_REVOLUTION = 28.0;
        public static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_PULSE_PER_REVOLUTION;

        public static final double MAX_TRANSLATIONAL_SPEED = 60.0;
        public static final double MAX_ROTATIONAL_SPEED = 2.0 * MAX_TRANSLATIONAL_SPEED / TRACK_WIDTH;
        public static final double MAX_TRANSLATIONAL_ACCELERATION = 200.0;
        public static final double MAX_ROTATIONAL_ACCELERATION = 20.0;

        public static final DifferentialDriveConfig DIFFERENTIAL_DRIVE_CONFIG = new DifferentialDriveConfig()
            .setLeftMotorName(LEFT_MOTOR_NAME)
            .setLeftMotorDirection(LEFT_MOTOR_DIRECTION)
            .setRightMotorName(RIGHT_MOTOR_NAME)
            .setRightMotorDirection(RIGHT_MOTOR_DIRECTION)
            .setKinematics(KINEMATICS)
            .setMaxTranslationalSpeed(MAX_TRANSLATIONAL_SPEED)
            .setMaxRotationalSpeed(MAX_ROTATIONAL_SPEED)
            .setMaxTranslationalAccel(MAX_TRANSLATIONAL_ACCELERATION)
            .setMaxRotationalAccel(MAX_ROTATIONAL_ACCELERATION)
            .setWheelCircumference(WHEEL_CIRCUMFERENCE)
            .setTicksPerRevolution(TICKS_PER_REVOLUTION)
            .setMotorVelocityPID(new PIDFCoefficients(1.0,0.0,0.0,13.0))
            .setTranslationalPID(new PIDFCoefficients(1.0,0.0,0.0,0.0))
            .setRotationalPID(new PIDFCoefficients(1.0,0.0,0.0,0.0));

    }

    public static class Shooter {

        public static final String SHOOTER_NAME = "shooter";

        //Change direction to fit needs of the robot
        public static final DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double GEAR_RATIO = 20;

        public static final double WHEEL_CIRCUMFERENCE = (2.0 * Math.PI) * 4.0;

        public static final double ENCODER_PULSE_PER_REVOLUTION = 7.0;

        public static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_PULSE_PER_REVOLUTION;

        public static final double MAX_VELOCITY = 50;

        public static final ShooterConfig CONFIG = new ShooterConfig(SHOOTER_NAME, SHOOTER_DIRECTION, WHEEL_CIRCUMFERENCE, ENCODER_PULSE_PER_REVOLUTION, GEAR_RATIO, MAX_VELOCITY);
    }

    public static class ExtraShooter {

        public static final String FIRST_SHOOTER_NAME = "FirstShooter";

        public static final String SECOND_SHOOTER_NAME = "SecondShooter";

        //Change direction to fit needs of the robot
        public static final DcMotorSimple.Direction FIRST_SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final DcMotorSimple.Direction SECOND_SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double GEAR_RATIO = 20;

        public static final double WHEEL_CIRCUMFERENCE = (2.0 * Math.PI) * 4.0;

        public static final double ENCODER_PULSE_PER_REVOLUTION = 7.0;

        public static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_PULSE_PER_REVOLUTION;

        public static final double MAX_VELOCITY = 50;

        public static final ExtraShooterConfig CONFIG = new ExtraShooterConfig(FIRST_SHOOTER_NAME, SECOND_SHOOTER_NAME, FIRST_SHOOTER_DIRECTION, SECOND_SHOOTER_DIRECTION, WHEEL_CIRCUMFERENCE, ENCODER_PULSE_PER_REVOLUTION, GEAR_RATIO, MAX_VELOCITY);
    }

    public static class Loader {
        public static final String LOADER_NAME = "Loader";

        public static final double START_POSITION = 0.0;

        public static final double LOAD_POSITION = 0.5;

        public static final double SHOOT_POSITION = 1.0;
    }
}
