package frc.robot;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
    private Transform3d transform;
    private AprilTagPoseEstimator estimator;

    private CvSink cvSink;
    private CvSource debugStream;

    public Camera(int deviceNumber, CameraType type, Transform3d transform) {
        this.transform = transform;
        this.estimator = new AprilTagPoseEstimator(type.getConfig());

        UsbCamera usb = CameraServer.startAutomaticCapture(deviceNumber);
        usb.setResolution(640, 480);
        this.cvSink = CameraServer.getVideo();
        this.debugStream = CameraServer.putVideo("Detected " + deviceNumber, 640, 480);
    }

    public AprilTagPoseEstimator getEstimator() {
        return this.estimator;
    }

    public CvSink getCvSink() {
        return this.cvSink;
    }

    public CvSource debugStream() {
        return this.debugStream;
    }

    public enum CameraType {
        LifeCam;

        Config getConfig() {
            // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
            switch (this) {
                case LifeCam:
                    return new AprilTagPoseEstimator.Config(
                            0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
                default:
                    throw new Error("Unknown config for " + this);
            }
        }
    }
}
