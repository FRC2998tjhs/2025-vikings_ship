package frc.robot;

import java.util.Comparator;
import java.util.Optional;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldRelativeMovement {
    private RobotRelativeMovement robotRelative;
    private RobotTransform transform;
    private AprilTags aprilTags;

    private PIDController turningPid = new PIDController(0.005, 0, 0);
    private PIDController orientPid = new PIDController(1.5, 0.2, 0);

    public Alignment leftOfReef;
    public Alignment rightOfReef;
    public Alignment pickup;

    public FieldRelativeMovement(RobotRelativeMovement robotRelative, RobotTransform transform, AprilTags aprilTags,
            Camera frontCamera, Camera backCamera) {
        this.robotRelative = robotRelative;
        this.transform = transform;
        this.aprilTags = aprilTags;

        this.leftOfReef = new Alignment(Tunable.leftOfReefAprilTagInCamera, frontCamera, 30, 60);
        this.rightOfReef = new Alignment(Tunable.rightOfReefAprilTagInCamera, frontCamera, 30, 60);
        this.pickup = new Alignment(Tunable.pickupAprilTagCamera, backCamera, 45, 90);

    }

    public void setDesiredState(Vector2 movementInput, Rotation2d direction) {
        Rotation2d measured = transform.getRotation();

        double turn = 0;
        if (direction != null) {
            var error = direction.minus(measured);
            turn = turningPid.calculate(error.getDegrees(), 0);
        }

        // System.out.println(movementInput);
        Vector2 robotMovement = robotMovement(movementInput);
        // System.out.println(robotMovement);
        robotRelative.setDesiredState(robotMovement, turn);
    }

    private Vector2 robotMovement(Vector2 fieldMovement) {
        Rotation2d measured = transform.getRotation();

        return fieldMovement.rotate(measured.minus(Rotation2d.kCCW_90deg).times(-1).getRadians());
    }

    public Optional<Double> alignTo(Alignment alignment,
            Vector2 otherwiseMove) {
        var measured = VikingMath.normalize(transform.getRotation());
        var angleWithLeastError = alignment.angles().min(Comparator.comparing(a -> {
            return Math.abs(a.minus(measured).getDegrees());
        }));

        var detection = aprilTags.seen().stream().filter(d -> d.camera == alignment.camera).findAny();
        if (detection.isEmpty()) {
            setDesiredState(otherwiseMove, angleWithLeastError.get());
            return Optional.empty();
        }
        var d = detection.get();

        var detectionCenter = d.centerCameraSpace();
        var error = new Vector2(
                alignment.cameraTarget.x - detectionCenter.x,
                detectionCenter.y - alignment.cameraTarget.y);
        var multiplyBy = VikingMath.clamp(orientPid.calculate(error.getMagnitude(), 0),
                -Tunable.maxAlignToAprilTagTurnSpeed, Tunable.maxAlignToAprilTagTurnSpeed);
        var movement = error.copy().multiply(multiplyBy);

        var turn = angleWithLeastError.map(a -> a.minus(measured).getDegrees() * -0.01).orElse((double) 0);

        robotRelative.setDesiredState(movement, turn);
        return Optional.of(error.getMagnitude());
    }

    public void turnRelative(Vector2 movement, double turn) {
        robotRelative.setDesiredState(robotMovement(movement), turn);
    }

    public void setForward() {
        transform.setCurrentAs(Rotation2d.kCCW_90deg);
    }

    public void setBackward() {
        transform.setCurrentAs(Rotation2d.kCW_90deg);
    }

    public void stop() {
        robotRelative.stop();
    }

    public class Alignment {
        public Vector2 cameraTarget;
        public Camera camera;
        public int anglesOffset;
        public int anglesDelta;

        public Alignment(Vector2 cameraTarget, Camera camera, int anglesOffset, int anglesDelta) {
            this.cameraTarget = cameraTarget;
            this.camera = camera;
            this.anglesOffset = anglesOffset;
            this.anglesDelta = anglesDelta;
        }

        Stream<Rotation2d> angles() {
            return IntStream.iterate(0, i -> i + 1)
                    .mapToObj(i -> Rotation2d.fromDegrees(this.anglesDelta * i + this.anglesOffset))
                    .takeWhile(angle -> angle.getDegrees() < 360)
                    .map(angle -> VikingMath.normalize(angle));
        }
    }

    public void rotateToBackward() {
        setDesiredState(new Vector2(), Rotation2d.kCW_90deg);
    }
}
