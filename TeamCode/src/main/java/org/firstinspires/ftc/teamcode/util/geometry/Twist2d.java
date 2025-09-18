package org.firstinspires.ftc.teamcode.util.geometry;

public class Twist2d {

    private final double dx;      // robot-relative X displacement (inches)
    private final double dy;      // robot-relative Y displacement (inches)
    private final double dtheta;  // change in rotation (radians)

    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public double getDx() {
        return dx;
    }

    public double getDy() {
        return dy;
    }

    public double getDtheta() {
        return dtheta;
    }

    public Translation2d getTranslation() {
        return new Translation2d(dx, dy);
    }

    public Rotation2d getRotation() {
        return new Rotation2d(dtheta);
    }

    public Pose2d asPose() {
        return new Pose2d(getTranslation(), getRotation());
    }
}
