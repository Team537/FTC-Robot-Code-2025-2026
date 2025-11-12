package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;

public interface PoseEstimator {

    public Pose2d getEstimatedPose();
    public void resetPose(Pose2d pose, Rotation2d currentGyroHeading);
    public void resetPose(Pose2d pose);


}
