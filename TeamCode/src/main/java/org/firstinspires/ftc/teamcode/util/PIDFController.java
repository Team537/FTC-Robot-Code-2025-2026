package org.firstinspires.ftc.teamcode.util;

import java.util.function.Supplier;

/**
 * PIDFController — A PID controller with optional feedforward term.
 *
 * Supports:
 * - Constant or dynamic (Supplier-based) feedforward
 * - Continuous input (for angles)
 * - Output and integral clamping
 * - Built-in DeltaTime tracking
 */
public class PIDFController {
    private double kP, kI, kD;
    private Supplier<Double> kF; // feedforward can be constant or dynamic

    private double integral = 0.0;
    private double prevError = 0.0;
    private boolean firstRun = true;

    private double minInput = 0.0;
    private double maxInput = 0.0;
    private boolean continuous = false;

    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private double integralMin = -1.0;
    private double integralMax = 1.0;

    private final DeltaTime deltaTime;

    /** Constructor with no feedforward (F = 0). */
    public PIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, () -> 0.0);
    }

    /** Constructor with constant feedforward. */
    public PIDFController(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, () -> kF);
    }

    /** Constructor with Supplier-based feedforward (for dynamic values). */
    public PIDFController(double kP, double kI, double kD, Supplier<Double> kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.deltaTime = new DeltaTime();
    }

    /** Constructor from FTC PIDFCoefficients (includes F term). */
    public PIDFController(PIDFCoefficients coeffs) {
        this(coeffs.kP, coeffs.kI, coeffs.kD, coeffs.kF);
    }

    /** Enable continuous input (e.g., angles). */
    public void enableContinuousInput(double minInput, double maxInput) {
        this.continuous = true;
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    /** Disable continuous input. */
    public void disableContinuousInput() {
        this.continuous = false;
    }

    /** Set output clamp range (e.g., motor power). */
    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    /** Set integral clamping range. */
    public void setIntegralRange(double min, double max) {
        this.integralMin = min;
        this.integralMax = max;
    }

    /** Reset integrator and error history. */
    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        firstRun = true;
        deltaTime.reset();
    }

    /** Returns the most recent feedforward value. */
    public double getF() {
        return kF.get();
    }

    /** Allows dynamically swapping the feedforward source. */
    public void setF(Supplier<Double> newF) {
        this.kF = newF;
    }

    /** Calculate PIDF output automatically using DeltaTime. */
    public double calculate(double measurement, double setpoint) {
        double dt = deltaTime.getDeltaTime();
        return calculate(measurement, setpoint, dt);
    }

    /** Calculate PIDF output with manually supplied dt. */
    public double calculate(double measurement, double setpoint, double dt) {
        double error = setpoint - measurement;

        if (continuous) {
            double range = maxInput - minInput;
            error = wrapError(error, range);
        }

        // PID terms
        double P = kP * error;

        integral += error * dt;
        integral = clamp(integral, integralMin, integralMax);
        double I = kI * integral;

        double derivative = 0.0;
        if (!firstRun) {
            derivative = (error - prevError) / dt;
        }
        double D = kD * derivative;

        // Feedforward
        double F = kF.get();

        prevError = error;
        firstRun = false;

        // Final output with clamping
        return clamp(P + I + D + F, minOutput, maxOutput);
    }

    /** Wrap error into [-range/2, range/2] if continuous input is enabled. */
    private double wrapError(double error, double range) {
        error %= range;
        if (error > range / 2.0) error -= range;
        else if (error < -range / 2.0) error += range;
        return error;
    }

    /** Clamp a value between min and max. */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
