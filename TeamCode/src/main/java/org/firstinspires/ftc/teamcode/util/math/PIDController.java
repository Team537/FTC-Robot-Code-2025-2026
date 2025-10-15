package org.firstinspires.ftc.teamcode.util.math;

public class PIDController {
    private double kP, kI, kD;

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

    // Internal DeltaTime instance
    private final DeltaTime deltaTime;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.deltaTime = new DeltaTime();
    }

    /** Enable continuous input (e.g., for angles). */
    public void enableContinuousInput(double minInput, double maxInput) {
        this.continuous = true;
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    /** Disable continuous input. */
    public void disableContinuousInput() {
        this.continuous = false;
    }

    /** Set the output range (e.g., motor powers). */
    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    /** Set the integral accumulator clamp to avoid windup. */
    public void setIntegralRange(double min, double max) {
        this.integralMin = min;
        this.integralMax = max;
    }

    /** Reset the controller (clears integral and previous error). */
    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        firstRun = true;
        deltaTime.reset();
    }

    /**
     * Calculate PID output automatically using DeltaTime.
     */
    public double calculate(double measurement, double setpoint) {
        double dt = deltaTime.getDeltaTime();
        return calculate(measurement, setpoint, dt);
    }

    /**
     * Calculate PID output with manually supplied dt (optional).
     */
    public double calculate(double measurement, double setpoint, double dt) {
        double error = setpoint - measurement;

        if (continuous) {
            double range = maxInput - minInput;
            error = wrapError(error, range);
        }

        // Proportional
        double P = kP * error;

        // Integral with clamping
        integral += error * dt;
        integral = clamp(integral, integralMin, integralMax);
        double I = kI * integral;

        // Derivative
        double derivative = 0.0;
        if (!firstRun) {
            derivative = (error - prevError) / dt;
        }
        double D = kD * derivative;

        prevError = error;
        firstRun = false;

        // Final output with clamping
        return clamp(P + I + D, minOutput, maxOutput);
    }

    /** Internal helper: wrap error into [-range/2, range/2]. */
    private double wrapError(double error, double range) {
        error %= range;
        if (error > range / 2.0) {
            error -= range;
        } else if (error < -range / 2.0) {
            error += range;
        }
        return error;
    }

    /** Internal helper: clamp a value. */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
