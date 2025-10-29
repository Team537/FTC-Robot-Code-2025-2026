package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.math.PIDFCoefficients;

/**
 * Base configuration for any holonomic drivetrain (Mecanum, Swerve, etc.).
 * Contains controller coefficients and physical constants common across all implementations.
 */
public class HolonomicDriveConfig {

    // --- Controllers ---
    /** PIDF controller for translational (x/y) motion. */
    public PIDFCoefficients translationalPID = new PIDFCoefficients();

    /** PIDF controller for rotational (heading) control. */
    public PIDFCoefficients rotationalPID = new PIDFCoefficients();

    // --- Physical constants ---
    /** Maximum translational speed in inches per second. */
    public double maxTranslationalSpeed = 0.0;

    /** Maximum rotational speed in radians per second. */
    public double maxRotationalSpeed = 0.0;

    /** Maximum translational acceleration in inches per second squared. */
    public double maxTranslationalAccel = 0.0;

    /** Maximum rotational acceleration in radians per second squared. */
    public double maxRotationalAccel = 0.0;

    // --- Fluent setters ---
    public HolonomicDriveConfig setTranslationalPID(PIDFCoefficients pid) {
        this.translationalPID = pid;
        return this;
    }

    public HolonomicDriveConfig setRotationalPID(PIDFCoefficients pid) {
        this.rotationalPID = pid;
        return this;
    }

    public HolonomicDriveConfig setMaxTranslationalSpeed(double s) {
        this.maxTranslationalSpeed = s;
        return this;
    }

    public HolonomicDriveConfig setMaxRotationalSpeed(double s) {
        this.maxRotationalSpeed = s;
        return this;
    }

    public HolonomicDriveConfig setMaxTranslationalAccel(double a) {
        this.maxTranslationalAccel = a;
        return this;
    }

    public HolonomicDriveConfig setMaxRotationalAccel(double a) {
        this.maxRotationalAccel = a;
        return this;
    }
}
