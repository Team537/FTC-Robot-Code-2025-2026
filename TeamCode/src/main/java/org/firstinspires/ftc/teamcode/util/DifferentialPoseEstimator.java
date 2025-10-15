package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Twist2d;

public class DifferentialPoseEstimator {

    private final DifferentialDriveKinematics kinematics;
    private Pose2d estimatedPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
    private Rotation2d gyroOffset = new Rotation2d(0.0);

    private double[] lastWheelPositions = new double[2]; // [left, right] in inches

    public DifferentialPoseEstimator(DifferentialDriveKinematics kinematics, Pose2d initialPose) {
        this.kinematics = kinematics;
        this.estimatedPose = initialPose;
    }

    /**
     * Reset the estimator’s internal pose (full reset).
     */
    public void resetPose(Pose2d newPose, Rotation2d currentGyroHeading) {
        this.estimatedPose = newPose;
        this.gyroOffset = newPose.getRotation().minus(currentGyroHeading);
        lastWheelPositions[0] = 0.0;
        lastWheelPositions[1] = 0.0;
    }

    /**
     * Update the pose from left/right wheel encoder distances + gyro heading.
     *
     * @param currentWheelPositions double[2] left/right distances in inches
     * @param currentGyroHeading    gyro rotation reading
     */
    public void update(double[] currentWheelPositions, Rotation2d currentGyroHeading) {
        if (currentWheelPositions.length != 2) {
            throw new IllegalArgumentException("Expected 2 wheel positions: [left, right]");
        }

        // 1. Compute wheel deltas
        double leftDelta = currentWheelPositions[0] - lastWheelPositions[0];
        double rightDelta = currentWheelPositions[1] - lastWheelPositions[1];

        lastWheelPositions[0] = currentWheelPositions[0];
        lastWheelPositions[1] = currentWheelPositions[1];

        // 2. Convert to robot-relative twist
        Twist2d twist = kinematics.fromWheelDeltas(new double[]{leftDelta, rightDelta});

        // 3. Integrate twist to update pose
        estimatedPose = estimatedPose.exp(twist);

        // 4. Fuse gyro heading + offset
        Rotation2d fusedHeading = currentGyroHeading.plus(gyroOffset);
        estimatedPose = new Pose2d(estimatedPose.getTranslation(), fusedHeading);
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }
}
