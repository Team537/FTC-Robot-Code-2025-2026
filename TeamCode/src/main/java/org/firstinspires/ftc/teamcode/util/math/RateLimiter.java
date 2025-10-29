package org.firstinspires.ftc.teamcode.util.math;

public class RateLimiter {

    // Value whose rate will be limited
    private double value;

    // The rate at which the value will be limited (in units per second)
    private double rate;

    // DeltaTime instance to track elapsed time between updates
    private final DeltaTime deltaTime;

    // The maximum delta time allowed by the rate limiter
    private double maxDeltaTime;

    /**
     * Creates a rate limiter.
     *
     * @param initialValue the initial value to be set
     * @param rate         the maximum rate of change (units per second)
     * @param maxDeltaTime the maximum delta time allowed; use Double.MAX_VALUE for
     *                     no restriction
     */
    public RateLimiter(double initialValue, double rate, double maxDeltaTime) {
        this.value = initialValue;
        this.rate = rate;
        this.maxDeltaTime = maxDeltaTime;
        this.deltaTime = new DeltaTime(); // tracks elapsed time automatically
    }

    /**
     * Creates a rate limiter with no restriction on maximum delta time.
     *
     * @param initialValue the initial value to be set
     * @param rate         the maximum rate of change (units per second)
     */
    public RateLimiter(double initialValue, double rate) {
        this(initialValue, rate, Double.MAX_VALUE);
    }

    /**
     * Updates the value, limiting the rate of change.
     *
     * @param targetValue the desired value to approach
     * @return the new, rate-limited value
     */
    public double update(double targetValue) {
        double dt = deltaTime.getDeltaTime(); // time since last call in seconds

        // Enforce maximum delta time
        if (dt > maxDeltaTime) {
            dt = maxDeltaTime;
        }

        // Calculate the maximum change allowed
        double maxChange = rate * dt;

        // Limit the change to the target value
        if (targetValue > value) {
            value = Math.min(value + maxChange, targetValue);
        } else if (targetValue < value) {
            value = Math.max(value - maxChange, targetValue);
        }

        return value;
    }

    /** @return the current value of the rate limiter. */
    public double getValue() {
        return value;
    }

    /** Sets the current value directly. */
    public void setValue(double value) {
        this.value = value;
        this.deltaTime.reset(); // avoid big jump on next update
    }

    /** Sets a new rate for the rate limiter. */
    public void setRate(double rate) {
        this.rate = rate;
    }

    /** Sets a new maximum delta time for the rate limiter. */
    public void setMaxDeltaTime(double maxDeltaTime) {
        this.maxDeltaTime = maxDeltaTime;
    }
}
