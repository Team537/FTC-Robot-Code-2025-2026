package org.firstinspires.ftc.teamcode.util.geometry;

public class Pose3d {
    private final Translation3d translation;
    private final Rotation3d rotation;

    public static final Pose3d ZERO = new Pose3d(Translation3d.ZERO, Rotation3d.ZERO);

    public Pose3d(Translation3d t, Rotation3d r) {
        this.translation = t; this.rotation = r;
    }

    public Translation3d getTranslation() { return translation; }
    public Rotation3d getRotation() { return rotation; }

    public Pose3d plus(Transform3d transform) {
        Translation3d newTranslation = translation.plus(rotation.applyTo(transform.getTranslation()));
        Rotation3d newRotation = rotation.plus(transform.getRotation());
        return new Pose3d(newTranslation, newRotation);
    }

    public Transform3d minus(Pose3d other) {
        Translation3d deltaTrans = translation.minus(other.translation);
        Rotation3d deltaRot = rotation.minus(other.rotation);
        return new Transform3d(deltaTrans, deltaRot);
    }

    public Pose3d exp(Twist3d twist) {
// Simplified: linear translation + rotation
        Translation3d newTranslation = translation.plus(twist.getLinear());
        Rotation3d newRotation = rotation.plus(twist.getAngular());
        return new Pose3d(newTranslation, newRotation);
    }
}
