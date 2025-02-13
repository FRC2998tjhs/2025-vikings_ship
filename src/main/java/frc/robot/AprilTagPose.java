package frc.robot;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagPose {
    public AprilTagDetection detection;
    public AprilTagPoseEstimate pose;

    private AprilTagPose(AprilTagDetection detection, AprilTagPoseEstimate pose) {
        this.detection = detection;
        this.pose = pose;
    }

    public static AprilTagPose fromDetection(AprilTagDetection d, AprilTagPoseEstimator estimator) {
        var pose = estimator.estimateOrthogonalIteration(d, 2);
        return new AprilTagPose(d, pose);
    }

    public Transform3d bestPose() {
        var whichPose = pose.error1 < pose.error2 ? pose.pose1 : pose.pose2;
        // return new Pose3d(whichPose.getTranslation(), whichPose.getRotation());
        return whichPose;
    }
}
