package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.util.geometry.ChassisVelocity2d;
import org.firstinspires.ftc.teamcode.util.geometry.Twist2d;

/**
 * Handles kinematics for a standard 4-wheel mecanum drivetrain.
 * Wheels are assumed in the order:
 * Front Left, Front Right, Back Left, Back Right
 */
public class MecanumDriveKinematics {

    private final double wheelBase;   // distance between front and back wheels (meters)
    private final double trackWidth;  // distance between left and right wheels (meters)

    /**
     * Constructor.
     * @param wheelBase distance from front to back wheels
     * @param trackWidth distance from left to right wheels
     */
    public MecanumDriveKinematics(double wheelBase, double trackWidth) {
        this.wheelBase = wheelBase;
        this.trackWidth = trackWidth;
    }

    /**
     * Converts robot-relative chassis velocity to individual wheel velocities.
     *
     * @param chassisVelocity robot-relative velocity (vx, vy, omega)
     * @return wheel velocities array: [frontLeft, frontRight, backLeft, backRight]
     */
    public double[] toWheelSpeeds(ChassisVelocity2d chassisVelocity) {
        double vx = chassisVelocity.getTranslationalVelocity().getX(); // forward
        double vy = chassisVelocity.getTranslationalVelocity().getY(); // right
        double omega = chassisVelocity.getRotationalVelocity();        // rad/s

        double radius = wheelBase / 2.0 + trackWidth / 2.0;

        double frontLeft  = vx - vy - omega * radius;
        double frontRight = vx + vy + omega * radius;
        double backLeft   = vx + vy - omega * radius;
        double backRight  = vx - vy + omega * radius;

        return new double[]{frontLeft, frontRight, backLeft, backRight};
    }

    /**
     * Converts wheel velocities to robot-relative chassis velocity.
     * @param wheelSpeeds wheel velocities: [frontLeft, frontRight, backLeft, backRight]
     * @return chassis velocity (translational + rotational)
     */
    public ChassisVelocity2d fromWheelSpeeds(double[] wheelSpeeds) {
        if (wheelSpeeds.length != 4) {
            throw new IllegalArgumentException("Expected 4 wheel speeds");
        }

        double frontLeft  = wheelSpeeds[0];
        double frontRight = wheelSpeeds[1];
        double backLeft   = wheelSpeeds[2];
        double backRight  = wheelSpeeds[3];

        double vx = (frontLeft + frontRight + backLeft + backRight) / 4.0;
        double vy = (-frontLeft + frontRight + backLeft - backRight) / 4.0;
        double omega = (-frontLeft + frontRight - backLeft + backRight) / (4.0 * (wheelBase / 2.0 + trackWidth / 2.0));

        return new ChassisVelocity2d(new Translation2d(vx, vy), omega);
    }

    /**
     * Compute robot-relative twist from four wheel displacements.
     * @param deltaInches array of wheel deltas [FL, FR, BL, BR] in inches
     * @return Twist2d representing dx, dy, dtheta in robot frame
     */
    public Twist2d fromWheelDeltas(double[] deltaInches) {
        if (deltaInches.length != 4) {
            throw new IllegalArgumentException("Expected 4 wheel deltas");
        }

        double fl = deltaInches[0];
        double fr = deltaInches[1];
        double bl = deltaInches[2];
        double br = deltaInches[3];

        // inverse kinematics for mecanum (robot-relative)
        double dx = (fl + fr + bl + br) / 4.0;  // forward/back
        double dy = (-fl + fr + bl - br) / 4.0; // strafe
        double dtheta = (-fl + fr - bl + br) / (4.0 * (trackWidth / 2.0 + wheelBase / 2.0)); // rotation

        return new Twist2d(dx, dy, dtheta);
    }

}