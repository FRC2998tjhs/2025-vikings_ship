package frc.robot;

import java.util.List;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Transform2d;

public class RobotTransform {
    private AHRS gyro;
    private Field field;

    public RobotTransform(AHRS gyro, AprilTags aprilTags, Field field) {
        this.gyro = gyro;
        this.field = field;

        aprilTags.onDetection((detection) -> {
            this.onDetection(detection);
        });
    }

    private void onDetection(List<AprilTagPose> poses) {
        for (AprilTagPose pose : poses) {
            var tagPosition = field.tagPosition(pose.detection.getId());
            if (tagPosition == null) {
                continue;
            }
            // tag's known position + pose estimate + camera's position on bot
            // System.out.println("Ambiguity " + pose.pose.getAmbiguity());
            // System.out.println("Found tag " + pose.detection.getId() + " at " + tagPosition);
        }
    }

    public Transform2d transform() {
        // System.out.println(gyro.getDisplacementZ());
        return new Transform2d();
    }
}
