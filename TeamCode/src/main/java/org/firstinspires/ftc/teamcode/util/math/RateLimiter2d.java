package org.firstinspires.ftc.teamcode.util.math;

import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;

public class RateLimiter2d {

    // Value whose rate will be limited
    private Translation2d value;

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
    public RateLimiter2d(Translation2d initialValue, double rate, double maxDeltaTime) {
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
    public RateLimiter2d(Translation2d initialValue, double rate) {
        this(initialValue, rate, Double.MAX_VALUE);
    }

    /**
     * Updates the value, limiting the rate of change.
     *
     * @param targetValue the desired value to approach
     * @return the new, rate-limited value
     */
    public Translation2d update(Translation2d targetValue) {
        double dt = deltaTime.getDeltaTime(); // time since last call in seconds

        // Enforce maximum delta time
        if (dt > maxDeltaTime) {
            dt = maxDeltaTime;
        }

        // Calculate the maximum change allowed
        double maxChange = rate * dt;

        // Limit the change to the target value
        Translation2d difference = targetValue.minus(value);
        if (difference.magnitude() < maxChange) {
            value = targetValue;
        } else {
            value = value.plus(difference.normalize().times(maxChange));
        }

        return value;
    }

    /** @return the current value of the rate limiter. */
    public Translation2d getValue() {
        return value;
    }

    /** Sets the current value directly. */
    public void setValue(Translation2d value) {
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
