package frc.robot;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

public class Camera {
    private AprilTagPoseEstimator estimator;

    private CvSink cvSink;
    private CvSource debugStream;

    private Vector2 resolution;

    public Camera(int deviceNumber, CameraType type) {
        this.estimator = new AprilTagPoseEstimator(type.getConfig());

        UsbCamera usb = CameraServer.startAutomaticCapture(deviceNumber);
        this.resolution = new Vector2(640, 480);
        usb.setResolution((int) resolution.x, (int) resolution.y);
        this.cvSink = CameraServer.getVideo();
        this.debugStream = CameraServer.putVideo("Detected " + deviceNumber, (int) resolution.x, (int) resolution.y);
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

    public Vector2 getResolution() {
        return this.resolution;
    }

    public enum CameraType {
        LifeCam, Elp;

        Config getConfig() {
            // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
            switch (this) {
                case LifeCam:
                    return new AprilTagPoseEstimator.Config(
                            0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
                case Elp:
                    return new AprilTagPoseEstimator.Config(
                            0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
                default:
                    throw new Error("Unknown config for " + this);
            }
        }
    }
}
