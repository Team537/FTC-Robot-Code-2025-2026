package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Twist2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

/**
 * Handles kinematics for a standard differential (2-wheel) drivetrain.
 *
 * Provides both forward and inverse kinematics:
 *  - Converts chassis velocities (linear + angular) into left/right wheel speeds.
 *  - Converts wheel speeds back into chassis velocities.
 *  - Computes chassis twist from wheel displacements (for odometry).
 */
public class DifferentialDriveKinematics {

    private final double trackWidth; // distance between left and right wheels (inches)
    private final Translation2d pivotOffset; // distance from the robot's origin to its pivot (ideally on the x=0 line_

    /**
     * Constructs a DifferentialDriveKinematics instance.
     *
     * @param trackWidth distance between the centers of the left and right wheels (inches)
     * @param pivotOffset distance to the pivot point from the robot's origin
     */
    public DifferentialDriveKinematics(double trackWidth, Translation2d pivotOffset) {
        this.trackWidth = trackWidth;
        this.pivotOffset = pivotOffset;
    }

    /**
     * Converts a desired chassis velocity into individual wheel speeds.
     *
     * @param chassisVelocity robot-relative velocity (linear, angular)
     * @return wheel velocities array: [left, right]
     */
    public double[] toWheelSpeeds(ChassisVelocity2d chassisVelocity) {
        double linear = chassisVelocity.getTranslationalVelocity().getX(); // forward velocity
        double angular = chassisVelocity.getRotationalVelocity();          // radians/sec (CCW positive)

        double left = linear - (angular * trackWidth / 2.0);
        double right = linear + (angular * trackWidth / 2.0);

        return new double[]{left,right};
    }

    /**
     * Converts wheel velocities to chassis velocity.
     *
     * @param wheelSpeeds wheel velocities: [left, right]
     * @return ChassisVelocity2d representing linear + angular velocity
     */
    public ChassisVelocity2d fromWheelSpeeds(double[] wheelSpeeds) {
        double left = wheelSpeeds[0];
        double right = wheelSpeeds[1];

        double linear = (left + right) / 2.0;
        double angular = (right - left) / trackWidth;

        return new ChassisVelocity2d(new Translation2d(linear, 0.0), angular);
    }

    /**
     * Computes robot-relative twist from wheel displacements (for odometry integration).
     *
     * @param deltaInches array of wheel deltas [left, right] in inches
     * @return Twist2d representing dx, dy, dtheta (robot-relative)
     */
    public Twist2d fromWheelDeltas(double[] deltaInches) {
        if (deltaInches.length != 2) {
            throw new IllegalArgumentException("Expected 2 wheel deltas: [left, right]");
        }

        double leftDelta = deltaInches[0];
        double rightDelta = deltaInches[1];

        double dtheta = (rightDelta - leftDelta) / trackWidth;
        double dx = (leftDelta + rightDelta) / 2.0;

        // Base translation (robot center)
        Translation2d deltaTranslation = new Translation2d(dx, 0.0);

        // Apply geometric correction if the odometry origin is offset
        Translation2d rotatedOffset = pivotOffset.rotateBy(new Rotation2d(dtheta));
        Translation2d offsetDelta = rotatedOffset.minus(pivotOffset);
        deltaTranslation = deltaTranslation.plus(offsetDelta);

        return new Twist2d(deltaTranslation.getX(), deltaTranslation.getY(), dtheta);
    }

    /**
     * Returns the track width used for kinematic calculations.
     *
     * @return track width (inches)
     */
    public double getTrackWidth() {
        return trackWidth;
    }
}
