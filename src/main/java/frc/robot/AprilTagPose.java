package frc.robot;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;

public class AprilTagPose {
    private AprilTagDetection detection;
    private AprilTagPoseEstimate pose;

    private AprilTagPose(AprilTagDetection detection, AprilTagPoseEstimate pose) {
        this.detection = detection;
        this.pose = pose;
    }

    public static AprilTagPose fromDetection(AprilTagDetection d, AprilTagPoseEstimator estimator) {
        var pose = estimator.estimateOrthogonalIteration(d, 2);
        return new AprilTagPose(d, pose);
    }
}
