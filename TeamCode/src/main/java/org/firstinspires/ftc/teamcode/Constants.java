package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.MecanumDriveKinematics;
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

        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

        public static final String IMU_NAME = "imu";

        public static final double WHEEL_BASE = 10.0;
        public static final double TRACK_WIDTH = 10.0;
        public static final MecanumDriveKinematics KINEMATICS = new MecanumDriveKinematics(WHEEL_BASE, TRACK_WIDTH);
        public static final double WHEEL_CIRCUMFERENCE = (2.0 * Math.PI) * 4.0;

        public static final double GEAR_RATIO = 19.2;
        public static final double ENCODER_PULSE_PER_REVOLUTION = 7.0;
        public static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_PULSE_PER_REVOLUTION;

        public static final double MAX_TRANSLATIONAL_SPEED = 50; // inches per second
        public static final double MAX_ROTATIONAL_SPEED = (MAX_TRANSLATIONAL_SPEED) / (WHEEL_BASE / 2.0 + TRACK_WIDTH / 2.0);
        public static final double MAX_WHEEL_SPEED = MAX_TRANSLATIONAL_SPEED * Math.sqrt(2.0);

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
    }

}
