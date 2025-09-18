package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Twist2d;

public class MecanumDriveSubsystem extends HolonomicDriveSubsystem {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    private IMU imu;

    private int[] lastWheelPositions = new int[4]; // FL, FR, BL, BR

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {

        super();
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, Constants.Drive.FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, Constants.Drive.FRONT_RIGHT_MOTOR_NAME);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, Constants.Drive.BACK_LEFT_MOTOR_NAME);
        backRightMotor = hardwareMap.get(DcMotorEx.class, Constants.Drive.BACK_RIGHT_MOTOR_NAME);
        imu = hardwareMap.get(IMU.class, Constants.Drive.IMU_NAME);

        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    }

    public void periodic() {
        super.periodic();
    }

    @Override
    public void setMotors(ChassisVelocity2d chassisVelocity) {
        // Use your kinematics class to get individual wheel speeds (in inches/sec)
        double[] wheelSpeedsInchesPerSec = Constants.Drive.KINEMATICS.toWheelSpeeds(chassisVelocity);

        // Convert inches/sec → ticks/sec
        double ticksPerInch = Constants.Drive.TICKS_PER_REVOLUTION / Constants.Drive.WHEEL_CIRCUMFERENCE;
        double[] wheelSpeedsTicksPerSec = new double[4];
        for (int i = 0; i < 4; i++) {
            wheelSpeedsTicksPerSec[i] = wheelSpeedsInchesPerSec[i] * ticksPerInch;
        }

        // Write velocity to motors
        frontLeftMotor.setVelocity(wheelSpeedsTicksPerSec[0]);
        frontRightMotor.setVelocity(wheelSpeedsTicksPerSec[1]);
        backLeftMotor.setVelocity(wheelSpeedsTicksPerSec[2]);
        backRightMotor.setVelocity(wheelSpeedsTicksPerSec[3]);
    }

    @Override
    public void updateOdometry() {

        int[] currentPositions = new int[] {
            frontLeftMotor.getCurrentPosition(),
            frontRightMotor.getCurrentPosition(),
            backLeftMotor.getCurrentPosition(),
            backRightMotor.getCurrentPosition()
        };

        int[] deltaTicks = new int[4];
        for (int i = 0; i < 4; i++) {
            deltaTicks[i] = currentPositions[i] - lastWheelPositions[i];
            lastWheelPositions[i] = currentPositions[i]; // store for next update
        }

        double ticksPerRev = Constants.Drive.TICKS_PER_REVOLUTION;
        double wheelCircumference = Constants.Drive.WHEEL_CIRCUMFERENCE;
        double[] deltaInches = new double[4];
        for (int i = 0; i < 4; i++) {
            deltaInches[i] = deltaTicks[i] * wheelCircumference / ticksPerRev;
        }

        Twist2d robotDelta = Constants.Drive.KINEMATICS.fromWheelDeltas(deltaInches);

        Rotation2d heading = new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        Pose2d newPose = getRobotPose().exp(robotDelta).setRotation(heading);
        setRobotPose(newPose);

    }

}