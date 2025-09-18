package org.firstinspires.ftc.teamcode.util.geometry;

public class Rotation2d {
    private final double radians;

    public Rotation2d(double radians) {
        this.radians = normalize(radians);
    }

    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Math.toRadians(degrees));
    }

    public double getRadians() {
        return radians;
    }

    public double getDegrees() {
        return Math.toDegrees(radians);
    }

    public Rotation2d plus(Rotation2d other) {
        return new Rotation2d(this.radians + other.radians);
    }

    public Rotation2d minus(Rotation2d other) {
        return new Rotation2d(this.radians - other.radians);
    }

    public Rotation2d unaryMinus() {
        return new Rotation2d(-this.radians);
    }

    private double normalize(double angle) {
        // normalize to [-pi, pi)
        double a = angle % (2 * Math.PI);
        if (a >= Math.PI) a -= 2 * Math.PI;
        if (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(%.2f rad, %.2f°)", radians, getDegrees());
    }
}
