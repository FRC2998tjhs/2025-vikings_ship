package frc.robot;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagPose {
    public AprilTagDetection detection;
    public AprilTagPoseEstimate pose;
    public Camera camera;

    private AprilTagPose(AprilTagDetection detection, AprilTagPoseEstimate pose, Camera camera) {
        this.detection = detection;
        this.pose = pose;
        this.camera = camera;
    }

    public static AprilTagPose fromDetection(AprilTagDetection d, Camera camera) {
        var pose = camera.getEstimator().estimateOrthogonalIteration(d, 2);
        return new AprilTagPose(d, pose, camera);
    }

    public Transform3d bestPose() {
        var whichPose = pose.error1 < pose.error2 ? pose.pose1 : pose.pose2;
        return whichPose;
    }

    public double centerXCameraSpace() {
        return detection.getCenterX() / camera.getResolution().x;
    }
}
