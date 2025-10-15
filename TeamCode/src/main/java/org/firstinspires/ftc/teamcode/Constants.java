package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.MecanumDriveKinematics;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

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

    }

}
