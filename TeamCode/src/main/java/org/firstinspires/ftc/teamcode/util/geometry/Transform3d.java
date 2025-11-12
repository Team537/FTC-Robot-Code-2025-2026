package org.firstinspires.ftc.teamcode.util.geometry;

public class Transform3d {
    private final Translation3d translation;
    private final Rotation3d rotation;

    public Transform3d(Translation3d t, Rotation3d r) {
        this.translation = t; this.rotation = r;
    }

    public Translation3d getTranslation() { return translation; }
    public Rotation3d getRotation() { return rotation; }

    public Transform3d inverse() {
        Rotation3d invRot = rotation.unaryMinus();
        Translation3d invTrans = invRot.applyTo(translation.times(-1));
        return new Transform3d(invTrans, invRot);
    }

    public Transform3d plus(Transform3d other) {
        Translation3d newTrans = translation.plus(rotation.applyTo(other.translation));
        Rotation3d newRot = rotation.plus(other.rotation);
        return new Transform3d(newTrans, newRot);
    }
}

