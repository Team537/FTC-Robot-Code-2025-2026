package org.firstinspires.ftc.teamcode.util.geometry;

public class Transform2d {
    private final Translation2d translation;
    private final Rotation2d rotation;

    public Transform2d(Translation2d translation, Rotation2d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Transform2d(double x, double y, double radians) {
        this(new Translation2d(x, y), new Rotation2d(radians));
    }

    public Translation2d getTranslation() { return translation; }
    public Rotation2d getRotation() { return rotation; }

    /** Apply this transform to a pose */
    public Pose2d applyTo(Pose2d pose) {
        Translation2d rotated = translation.rotateBy(pose.getRotation());
        Translation2d newTranslation = pose.getTranslation().plus(rotated);
        Rotation2d newRotation = pose.getRotation().plus(rotation);
        return new Pose2d(newTranslation, newRotation);
    }

    /** Invert the transform (undo it) */
    public Transform2d inverse() {
        Rotation2d invRot = rotation.unaryMinus();
        Translation2d invTrans = translation.rotateBy(invRot).unaryMinus();
        return new Transform2d(invTrans, invRot);
    }

    public static final Transform2d ZERO = new Transform2d(Translation2d.ZERO,Rotation2d.ZERO);

    @Override
    public String toString() {
        return String.format("Transform2d(%s, %s)", translation, rotation);
    }
}