package org.firstinspires.ftc.teamcode.util.geometry;

public class Twist3d {
    private final Translation3d linear;
    private final Rotation3d angular;

    public Twist3d(Translation3d linear, Rotation3d angular) {
        this.linear = linear;
        this.angular = angular;
    }

    public Translation3d getLinear() {
        return linear;
    }

    public Rotation3d getAngular() {
        return angular;
    }

    public Twist3d plus(Twist3d other) {
        return new Twist3d(linear.plus(other.linear), angular.plus(other.angular));
    }

    public Twist3d times(double scalar) {
        return new Twist3d(linear.times(scalar), new Rotation3d(
            angular.getRoll() * scalar,
            angular.getPitch() * scalar,
            angular.getYaw() * scalar
        ));
    }

}
