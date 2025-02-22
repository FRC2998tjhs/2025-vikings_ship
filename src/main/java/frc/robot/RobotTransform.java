package frc.robot;

import java.util.List;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class RobotTransform {
    private AHRS gyro;
    private Rotation2d gyroOffset;

    private Field field;

    public RobotTransform(AHRS gyro, Rotation2d gyroOffset, AprilTags aprilTags, Field field) {
        this.gyro = gyro;
        this.gyroOffset = gyroOffset;

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
            var cameraInWorldBadAxes = tagPose.plus(cameraToAprilTag);

            // TODO(shelbyd): Why do we need to correct the axes?
            Rotation3d fixAxes = new Rotation3d(-Math.PI / 2, 0, 0);
            var cameraInWorld = cameraInWorldBadAxes.transformBy(new Transform3d(new Translation3d(), fixAxes));

            var cameraTranslation = cameraInWorld.getTranslation();
            System.out.println(cameraTranslation);
            // var cameraPosition = new Vector3(cameraTranslation.getX(), -cameraTranslation.getZ(), -cameraTranslation.getY());

            // var robotPosition = pose.camera.getTransform().inverse();
            // System.out.println(cameraPosition);

            // tag's known position + pose estimate + camera's position on bot
        }
    }

    public Rotation2d getRotation() {
        var robotRotation = gyro.getRotation2d().minus(gyroOffset);
        return robotRotation;
    }

    public void setCurrentAs(Rotation2d rotation) {
        this.gyroOffset = gyro.getRotation2d().minus(rotation);
    }
}
