package org.firstinspires.ftc.teamcode.util.geometry;

public class Pose2d {
    private final Translation2d translation;
    private final Rotation2d rotation;

    public Pose2d(Translation2d translation, Rotation2d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Pose2d(double x, double y, double radians) {
        this(new Translation2d(x, y), new Rotation2d(radians));
    }

    public Translation2d getTranslation() { return translation; }
    public Rotation2d getRotation() { return rotation; }

    public Pose2d setTranslation(Translation2d translation) {
        return new Pose2d(translation, this.rotation);
    }

    public Pose2d setRotation(Rotation2d rotation) {
        return new Pose2d(this.translation, rotation);
    }

    /** Apply a transform to this pose */
    public Pose2d plus(Transform2d transform) {
        return transform.applyTo(this);
    }

    /** Find the transform from another pose to this pose */
    public Transform2d minus(Pose2d other) {
        // Compute relative translation
        Translation2d diff = this.translation.minus(other.translation);
        Translation2d rotated = diff.rotateBy(other.rotation.unaryMinus());
        // Compute relative rotation
        Rotation2d rot = this.rotation.minus(other.rotation);
        return new Transform2d(rotated, rot);
    }

    /**
     * Apply a small robot-relative twist (dx, dy, dtheta) to this pose.
     * Returns a new Pose2d.
     */
    public Pose2d exp(Twist2d twist) {
        double dx = twist.getDx();
        double dy = twist.getDy();
        double dtheta = twist.getDtheta();

        double sinTheta, cosTheta;
        double newX, newY;

        if (Math.abs(dtheta) < 1e-9) {
            // small angle approximation
            newX = dx;
            newY = dy;
        } else {
            sinTheta = Math.sin(dtheta);
            cosTheta = Math.cos(dtheta);
            newX = (dx * sinTheta + dy * (1 - cosTheta)) / dtheta;
            newY = (dy * sinTheta - dx * (1 - cosTheta)) / dtheta;
        }

        // rotate the delta by the current pose heading
        double heading = rotation.getRadians();
        double cosHeading = Math.cos(heading);
        double sinHeading = Math.sin(heading);

        Translation2d fieldDelta = new Translation2d(
            newX * cosHeading - newY * sinHeading,
            newX * sinHeading + newY * cosHeading
        );

        Rotation2d newRotation = rotation.plus(new Rotation2d(dtheta));

        return new Pose2d(fieldDelta.plus(translation), newRotation);
    }

    /** Express this pose relative to another pose’s frame */
    public Pose2d relativeTo(Pose2d other) {
        return other.inverse().plus(this.minus(other));
    }

    /** Inverse of this pose (pose that would undo this one) */
    public Pose2d inverse() {
        Rotation2d invRot = rotation.unaryMinus();
        Translation2d invTrans = translation.rotateBy(invRot).unaryMinus();
        return new Pose2d(invTrans, invRot);
    }

    public static final Pose2d ZERO = new Pose2d(Translation2d.ZERO, Rotation2d.ZERO);

    @Override
    public String toString() {
        return String.format("Pose2d(%s, %s)", translation, rotation);
    }
}