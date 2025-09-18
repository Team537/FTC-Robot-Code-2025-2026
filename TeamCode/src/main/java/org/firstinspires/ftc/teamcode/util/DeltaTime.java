package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DeltaTime {

    private final ElapsedTime timer;
    private double lastTime;

    /**
     * Constructor: Initializes the timer with the current timestamp.
     */
    public DeltaTime() {
        timer = new ElapsedTime();
        lastTime = timer.seconds();
    }

    /**
     * Returns the elapsed time in seconds since the last call to getDeltaTime(),
     * and resets the reference time to the current timestamp.
     *
     * @return The elapsed time in seconds.
     */
    public double getDeltaTime() {
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        return deltaTime;
    }

    /**
     * Returns the elapsed time in seconds since the last reset
     * without resetting the reference timestamp.
     *
     * @return The elapsed time in seconds since the last update.
     */
    public double peekDeltaTime() {
        double currentTime = timer.seconds();
        return currentTime - lastTime;
    }

    /**
     * Resets the timer reference point to the current timestamp.
     */
    public void reset() {
        lastTime = timer.seconds();
    }
}
