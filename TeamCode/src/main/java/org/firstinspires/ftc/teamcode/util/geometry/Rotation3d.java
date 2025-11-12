package org.firstinspires.ftc.teamcode.util.geometry;

public class Rotation3d {

    private final double roll, pitch, yaw;

    public static final Rotation3d ZERO = new Rotation3d(0.0,0.0,0.0);

    public Rotation3d(double roll, double pitch, double yaw) {
        this.roll = roll; this.pitch = pitch; this.yaw = yaw;
    }

    public double getRoll() { return roll; }
    public double getPitch() { return pitch; }
    public double getYaw() {return yaw; }

    public Rotation3d plus(Rotation3d other) {
        return new Rotation3d(roll + other.roll, pitch + other.pitch, yaw + other.yaw);
    }

    public Rotation3d minus(Rotation3d other) {
        return new Rotation3d(roll - other.roll, pitch - other.pitch, yaw - other.yaw);
    }

    public Rotation3d unaryMinus() {
        return new Rotation3d(-roll,-pitch,-yaw);
    }

    public Translation3d applyTo(Translation3d t) {
        double cr = Math.cos(roll), sr = Math.sin(roll);
        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double cy = Math.cos(yaw), sy = Math.sin(yaw);

        double x = t.getX(), y = t.getY(), z = t.getZ();

        double nx = cy*(cp*x + sp*sr*y + sp*cr*z);
        double ny = -sy*(cp*x + sp*sr*y + sp*cr*z);
        double nz = -sp*x + cp*sr*y + cp*cr*z;

        return new Translation3d(nx, ny, nz);
    }

}
