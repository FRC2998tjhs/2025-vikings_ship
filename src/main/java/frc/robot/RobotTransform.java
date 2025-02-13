package frc.robot;

import java.util.List;

import org.dyn4j.geometry.Vector3;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

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

            var tagPose = new Pose3d(tagPosition.x, tagPosition.y, tagPosition.z, new Rotation3d());
            var cameraToAprilTag = pose.bestPose().inverse();
            var cameraInWorld = tagPose.plus(cameraToAprilTag);

            var cameraTranslation = cameraInWorld.getTranslation();
            var correctAxes = new Vector3(cameraTranslation.getX(), -cameraTranslation.getZ(),
                    -cameraTranslation.getY());
            // TODO(shelbyd): Why do we need to correct the axes?
            System.out.println(correctAxes);

            // tag's known position + pose estimate + camera's position on bot
        }
    }

    public Transform2d transform() {
        // System.out.println(gyro.getDisplacementZ());
        return new Transform2d();
    }
}
