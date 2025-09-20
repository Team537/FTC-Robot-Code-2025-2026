package org.firstinspires.ftc.teamcode.util.geometry;

public class Translation2d {
    private final double x;
    private final double y;

    public Translation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() { return x; }
    public double getY() { return y; }

    public double norm() {
        return Math.hypot(x, y);
    }

    public Translation2d plus(Translation2d other) {
        return new Translation2d(x + other.x, y + other.y);
    }

    public Translation2d minus(Translation2d other) {
        return new Translation2d(x - other.x, y - other.y);
    }

    public Translation2d times(double scalar) {
        return new Translation2d(x * scalar,y * scalar);
    }

    public Translation2d div(double scalar) {
        return new Translation2d(x / scalar, y / scalar);
    }

    public Translation2d rotateBy(Rotation2d rot) {
        double cos = Math.cos(rot.getRadians());
        double sin = Math.sin(rot.getRadians());
        return new Translation2d(
            x * cos - y * sin,
            x * sin + y * cos
        );
    }

    public Translation2d unaryMinus() {
        return new Translation2d(-x, -y);
    }

    public static final Translation2d ZERO = new Translation2d(0.0,0.0);

    @Override
    public String toString() {
        return String.format("Translation2d(x=%.2f, y=%.2f)", x, y);
    }

}