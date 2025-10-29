package org.firstinspires.ftc.teamcode.util.geometry;

public class ChassisVelocity2d {
    private final Translation2d translationalVelocity;
    private final double rotationalVelocity;

    /**
     * Creates a ChassisVelocity2d with a given translation and rotation speed.
     *
     * @param translationalVelocity velocity along x and y axes
     * @param rotationalVelocity    angular velocity (radians per second)
     */
    public ChassisVelocity2d(Translation2d translationalVelocity, double rotationalVelocity) {
        this.translationalVelocity = translationalVelocity;
        this.rotationalVelocity = rotationalVelocity;
    }

    /** @return the translational velocity (x and y components) */
    public Translation2d getTranslationalVelocity() {
        return translationalVelocity;
    }

    /** @return the rotational velocity (radians per second) */
    public double getRotationalVelocity() {
        return rotationalVelocity;
    }

    /**
     * Converts this velocity into a field-relative velocity using the given robot orientation.
     *
     * @param robotHeading current robot orientation as a Rotation2d
     * @return ChassisVelocity2d expressed in field coordinates
     */
    public ChassisVelocity2d toFieldRelative(Rotation2d robotHeading) {
        Translation2d fieldTranslational = translationalVelocity.rotateBy(robotHeading);
        return new ChassisVelocity2d(fieldTranslational, rotationalVelocity);
    }

    /**
     * Converts this velocity into a robot-relative velocity using the given robot orientation.
     *
     * @param robotHeading current robot orientation as a Rotation2d
     * @return ChassisVelocity2d expressed in robot coordinates
     */
    public ChassisVelocity2d toRobotRelative(Rotation2d robotHeading) {
        Translation2d robotTranslational = translationalVelocity.rotateBy(robotHeading.unaryMinus());
        return new ChassisVelocity2d(robotTranslational, rotationalVelocity);
    }

    public static final ChassisVelocity2d ZERO = new ChassisVelocity2d(Translation2d.ZERO, 0.0);

    @Override
    public String toString() {
        return String.format("ChassisVelocity2d{translation=%s, rotation=%.3f}", translationalVelocity, rotationalVelocity);
    }
}
