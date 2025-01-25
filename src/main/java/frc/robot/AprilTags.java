package frc.robot;

import java.util.ArrayList;
import java.util.List;

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

    private AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(
            0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
    private AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);

    public AprilTags() {
        var visionThread = new Thread(this::apriltagVisionThreadProc);
        visionThread.setDaemon(true);
        visionThread.start();
    }

    public List<AprilTagPose> seen() {
        return latest;
    }

    void apriltagVisionThreadProc() {
        var detector = new AprilTagDetector();
        // look for tag36h11, correct 1 error bit (hamming distance 1)
        // hamming 1 allocates 781KB, 2 allocates 27.4 MB, 3 allocates 932 MB
        // max of 1 recommended for RoboRIO 1, while hamming 2 is feasible on the
        // RoboRIO 2
        detector.addFamily("tag36h11", 1);

        // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
        // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)

        // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // Set the resolution
        camera.setResolution(640, 480);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);

        // Mats are very memory expensive. Lets reuse these.
        var mat = new Mat();
        var grayMat = new Mat();

        // Instantiate once
        ArrayList<Long> everSeen = new ArrayList<>();
        var outlineColor = new Scalar(0, 255, 0);
        var crossColor = new Scalar(0, 0, 255);

        // We'll output to NT
        NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
        IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
            }

            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

            AprilTagDetection[] detections = detector.detect(grayMat);

            // have not seen any tags yet
            everSeen.clear();

            var seenThisFrame = new ArrayList<AprilTagPose>();
            for (AprilTagDetection detection : detections) {
                // remember we saw this tag
                everSeen.add((long) detection.getId());
                seenThisFrame.add(AprilTagPose.fromDetection(detection, estimator));

                outputDebug(estimator, mat, outlineColor, crossColor, tagsTable, detection);
            }

            latest = seenThisFrame;

            // put list of tags onto dashboard
            pubTags.set(everSeen.stream().mapToLong(Long::longValue).toArray());

            // Give the output stream a new image to display
            outputStream.putFrame(mat);
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
}
