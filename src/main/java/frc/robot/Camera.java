package frc.robot;

import org.dyn4j.geometry.Vector2;
import org.opencv.core.Mat;

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
    private Mat mat;
    private Mat grayMat;

    private Vector2 resolution;

    public Camera(int deviceNumber, String name, CameraType type) {
        this.estimator = new AprilTagPoseEstimator(type.getConfig());
        this.resolution = new Vector2(620, 480);

        UsbCamera usb = CameraServer.startAutomaticCapture(deviceNumber);
        usb.setResolution((int) resolution.x, (int) resolution.y);
        usb.setFPS(1);

        this.cvSink = CameraServer.getVideo(usb);
        this.debugStream = CameraServer.putVideo("Detected " + name, (int) resolution.x, (int) resolution.y);
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

    public Mat getMat() {
        if (this.mat == null) {
            this.mat = new Mat();
        }
        return this.mat;
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

    public Mat getGrayMat() {
        if (this.grayMat == null) {
            this.grayMat = new Mat();
        }
        return this.grayMat;
    }
}
