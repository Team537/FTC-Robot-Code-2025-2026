package org.firstinspires.ftc.teamcode.util.math;

import java.util.function.Supplier;

/**
 * Simple container for PIDF coefficients (Proportional, Integral, Derivative, Feedforward).
 *
 * Supports:
 *  - easy construction and copying
 *  - tuning via Dashboard or JSON
 *  - string output for debugging
 */
public class PIDFCoefficients {

    public double kP;
    public double kI;
    public double kD;
    public Supplier<Double> kF;

    /** Default constructor (all zero). */
    public PIDFCoefficients() {
        this(0, 0, 0, 0);
    }

    /** Main constructor. */
    public PIDFCoefficients(double kP, double kI, double kD, double kF) {
        this(kP,kI,kD,() -> kF);
    }

    /** Main constructor. */
    public PIDFCoefficients(double kP, double kI, double kD, Supplier<Double> kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** Copy constructor. */
    public PIDFCoefficients(PIDFCoefficients other) {
        this(other.kP, other.kI, other.kD, other.kF);
    }

    /** Returns a copy of this PIDF. */
    public PIDFCoefficients copy() {
        return new PIDFCoefficients(this);
    }

    /** Fluent setters for tuning convenience. */
    public PIDFCoefficients setP(double kP) { this.kP = kP; return this; }
    public PIDFCoefficients setI(double kI) { this.kI = kI; return this; }
    public PIDFCoefficients setD(double kD) { this.kD = kD; return this; }
    public PIDFCoefficients setF(Supplier<Double> kF) { this.kF = kF; return this; }
    public PIDFCoefficients setF(double kF) {this.kF = () -> kF; return this; }

    /**
     * Applies a scaling factor to all terms — useful when adjusting PIDF to different units
     * or motor controller ranges.
     */
    public PIDFCoefficients scale(double factor) {
        this.kP *= factor;
        this.kI *= factor;
        this.kD *= factor;
        this.kF = () -> factor * kF.get();
        return this;
    }

    @Override
    public String toString() {
        return String.format("PIDF(kP=%.5f, kI=%.5f, kD=%.5f, kF=%.5f)", kP, kI, kD, kF);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof PIDFCoefficients)) return false;
        PIDFCoefficients other = (PIDFCoefficients) obj;
        return Double.compare(kP, other.kP) == 0 &&
            Double.compare(kI, other.kI) == 0 &&
            Double.compare(kD, other.kD) == 0 &&
            kF == other.kF;
    }

}
