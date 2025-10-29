package org.firstinspires.ftc.teamcode.util.math;

import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

public class PoseMeasurement2d {

    private Pose2d pose;
    private double deltaTime;
    private double[] stdDevs = {0.2,0.2,0.087};

    /**
     * Creates a Pose Measurement 2d with the statistics
     * @param pose the pose, in inches
     * @param deltaTime the timestamp, in seconds
     * @param stdDevs the standard deviations, {x (inches), y (inches), theta (radians}
     */
    public PoseMeasurement2d(Pose2d pose, double deltaTime, double[] stdDevs) {
        this.pose = pose;
        this.deltaTime = deltaTime;
        this.stdDevs = stdDevs;
    }

    /**
     * gets the pose
     * @return the pose in inches
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * gets the timestamp
     * @return the timestamp in seconds
     */
    public double getDeltaTime() {
        return deltaTime;
    }

    /**
     * gets the standard deviations
     * @return the standard deviations {x (inches), y (inches), theta (radians}
     */
    public double[] getStdDevs() {
        return stdDevs;
    }

}
