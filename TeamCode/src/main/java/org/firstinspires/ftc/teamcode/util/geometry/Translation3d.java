package org.firstinspires.ftc.teamcode.util.geometry;

public class Translation3d {

    private final double x, y, z;

    public static final Translation3d ZERO = new Translation3d(0.0,0.0,0.0);

    public Translation3d(double x, double y, double z) {
        this.x = x; this.y = y; this.z =z;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getZ() { return z; }

    public Translation3d plus(Translation3d other) {
        return new Translation3d(x + other.x, y + other.y, z + other.z);
    }

    public Translation3d minus(Translation3d other) {
        return new Translation3d(x - other.x, y - other.y, z - other.z);
    }

    public Translation3d times(double scalar) {
        return new Translation3d(x * scalar, y * scalar, z * scalar);
    }

    public Translation3d unaryMinus() {
        return new Translation3d(-x,-y,z);
    }

    public double magnitude() {
        return Math.sqrt(x*x + y*y + z*z);
    }

    public double distance(Translation3d other) {
        return this.minus(other).magnitude();
    }

    public Translation3d rotateBy(Rotation3d r) {
        return r.applyTo(this);
    }

}
