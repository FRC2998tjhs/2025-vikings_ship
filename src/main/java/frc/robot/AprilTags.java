package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTags {
    private List<AprilTagPose> latest = new ArrayList<AprilTagPose>();

    private ArrayList<Consumer<List<AprilTagPose>>> onDetectionConsumers = new ArrayList<>();
    private List<Camera> cameras;

    public AprilTags(List<Camera> cameras) {
        this.cameras = cameras;

        var visionThread = new Thread(this::apriltagVisionThreadProc);
        visionThread.setDaemon(true);
        visionThread.start();
    }

    public List<AprilTagPose> seen() {
        return latest;
    }

    void apriltagVisionThreadProc() {
        var detector = new AprilTagDetector();
        detector.addFamily("tag36h11", 1);

        var mat = new Mat();
        var grayMat = new Mat();

        ArrayList<Long> everSeen = new ArrayList<>();
        var outlineColor = new Scalar(0, 255, 0);
        var crossColor = new Scalar(0, 0, 255);

        NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
        IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

        while (!Thread.interrupted()) {
            var seenThisFrame = new ArrayList<AprilTagPose>();
            everSeen.clear();

            for (var camera : this.cameras) {
                var sink = camera.getCvSink();

                if (sink.grabFrame(mat) == 0) {
                    camera.debugStream().notifyError(sink.getError());
                    continue;
                }

                Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

                AprilTagDetection[] detections = detector.detect(grayMat);

                for (AprilTagDetection detection : detections) {
                    everSeen.add((long) detection.getId());
                    var estimator = camera.getEstimator();
                    seenThisFrame.add(AprilTagPose.fromDetection(detection, camera));

                    outputDebug(estimator, mat, outlineColor, crossColor, tagsTable, detection);
                }

                camera.debugStream().putFrame(mat);
            }

            latest = seenThisFrame;

            for (var consumer : onDetectionConsumers) {
                consumer.accept(latest);
            }

            pubTags.set(everSeen.stream().mapToLong(Long::longValue).toArray());
        }

        pubTags.close();
        detector.close();
    }

    private void outputDebug(AprilTagPoseEstimator estimator, Mat mat, Scalar outlineColor, Scalar crossColor,
            NetworkTable tagsTable, AprilTagDetection detection) {
        // draw lines around the tag
        for (var i = 0; i <= 3; i++) {
            var j = (i + 1) % 4;
            var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
            var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
            Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        // mark the center of the tag
        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

        // identify the tag
        Imgproc.putText(
                mat,
                Integer.toString(detection.getId()),
                new Point(cx + ll, cy),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                1,
                crossColor,
                3);

        // determine pose
        Transform3d pose = estimator.estimate(detection);

        // put pose into dashboard
        Rotation3d rot = pose.getRotation();
        tagsTable
                .getEntry("pose_" + detection.getId())
                .setDoubleArray(
                        new double[] {
                                pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ()
                        });
    }

    public void onDetection(Consumer<List<AprilTagPose>> onDetection) {
        onDetectionConsumers.add(onDetection);
    }
}
