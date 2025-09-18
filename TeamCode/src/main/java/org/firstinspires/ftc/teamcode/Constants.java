package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.MecanumDriveKinematics;

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

        public static final String IMU_NAME = "imu";

        public static final double WHEEL_BASE = 10.0;
        public static final double TRACK_WIDTH = 10.0;
        public static final MecanumDriveKinematics KINEMATICS = new MecanumDriveKinematics(WHEEL_BASE, TRACK_WIDTH);
        public static final double WHEEL_CIRCUMFERENCE = (2.0 * Math.PI) * 4.0;

        public static final double GEAR_RATIO = 19.2;
        public static final double ENCODER_PULSE_PER_REVOLUTION = 7.0;
        public static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_PULSE_PER_REVOLUTION;

        public static final double MAX_SPEED = 50;


    }

}
