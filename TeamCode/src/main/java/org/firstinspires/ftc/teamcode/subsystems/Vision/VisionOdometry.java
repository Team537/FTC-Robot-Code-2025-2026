package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.PoseEstimator;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.util.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.util.geometry.Translation3d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class VisionOdometry extends Subsystem {

    private List<AprilTagProcessor> tagProcessors;

    private List<Integer> validTagIDs;

    private PoseEstimator poseEstimator;


    public VisionOdometry(List<AprilTagProcessor> tagProcessors, List<Integer> validTagIDs, PoseEstimator poseEstimator) {
        this.tagProcessors = tagProcessors;
        this.validTagIDs = validTagIDs;
        this.poseEstimator = poseEstimator;
    }

    public void periodic() {

        List<AprilTagDetection> detections = new ArrayList<>();

        for (AprilTagProcessor tagProcessor : tagProcessors) {
            for (AprilTagDetection detection : tagProcessor.getDetections()) {
                if (validTagIDs.contains(detection.id)) {
                    detections.add(detection);
                }
            }
        }

        TelemetryManager.put("#Detections",detections.size());

        if (detections.isEmpty()) return;

        double minDistance = Double.MAX_VALUE;
        AprilTagDetection bestDetection = detections.get(0);
        for (AprilTagDetection detection : detections) {
            double distance = translation3dFromAprilTagFtcPose(detection.ftcPose).getTranslation().magnitude();
            if (minDistance > distance) {
                minDistance = distance;
                bestDetection = detection;
            }
        }

        Pose3d robotPose = pose3dFromAprilTagRobotPose(bestDetection.robotPose);
        poseEstimator.resetPose(
            new Pose2d(
                new Translation2d(
                    robotPose.getTranslation().getX(),
                    robotPose.getTranslation().getY()
                ),
                new Rotation2d(
                    robotPose.getRotation().getYaw()
                )
            )
        );

    }

    public Transform3d translation3dFromAprilTagFtcPose(AprilTagPoseFtc ftcPose) {
        return new Transform3d(
            new Translation3d(ftcPose.x,ftcPose.y,ftcPose.z),
            new Rotation3d(ftcPose.roll,ftcPose.pitch,ftcPose.yaw)
        );

    }

    public Pose3d pose3dFromAprilTagRobotPose(Pose3D rawPose) {
        return new Pose3d(
            new Translation3d(
                rawPose.getPosition().x,
                rawPose.getPosition().y,
                rawPose.getPosition().z
            ),
            new Rotation3d(
                rawPose.getOrientation().getRoll(AngleUnit.RADIANS),
                rawPose.getOrientation().getPitch(AngleUnit.RADIANS),
                rawPose.getOrientation().getYaw(AngleUnit.RADIANS)
            )
        );
    }

}
