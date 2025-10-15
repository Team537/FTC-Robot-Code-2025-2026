package org.firstinspires.ftc.teamcode.util.mecanum;

import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Twist2d;

public class MecanumPoseEstimator {

    private final MecanumDriveKinematics kinematics;
    private Pose2d estimatedPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
    private Rotation2d gyroOffset = new Rotation2d(0.0);

    private double[] lastWheelPositions = new double[4]; // ticks

    public MecanumPoseEstimator(MecanumDriveKinematics kinematics, Pose2d initialPose) {
        this.kinematics = kinematics;
        this.estimatedPose = initialPose;
    }

    /**
     * Reset the estimator’s internal pose (full reset).
     */
    public void resetPose(Pose2d newPose, Rotation2d currentGyroHeading) {
        this.estimatedPose = newPose;
        // Compute offset so that gyro reading matches newPose heading
        this.gyroOffset = newPose.getRotation().minus(currentGyroHeading);
        // also reset lastWheelPositions if needed
    }

    /**
     * Update from wheel encoders + gyro
     */
    public void update(double[] currentWheelPositions, Rotation2d currentGyroHeading) {
        // 1. Wheel deltas
        double[] deltas = new double[4];
        for (int i = 0; i < 4; i++) {
            deltas[i] = currentWheelPositions[i] - lastWheelPositions[i];
            lastWheelPositions[i] = currentWheelPositions[i];
        }

        // 2. Convert to chassis twist
        Twist2d twist = kinematics.fromWheelDeltas(deltas);

        // 3. Integrate twist
        estimatedPose = estimatedPose.exp(twist);

        // 4. Override orientation with gyro + offset
        Rotation2d fusedHeading = currentGyroHeading.plus(gyroOffset);
        estimatedPose = new Pose2d(estimatedPose.getTranslation(), fusedHeading);
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

}
