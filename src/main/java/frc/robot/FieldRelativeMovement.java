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
    private Camera frontCamera;
    private Camera backCamera;

    private PIDController turningPid = new PIDController(0.005, 0, 0);
    private PIDController orientPid = new PIDController(1.5, 0.2, 0);

    public FieldRelativeMovement(RobotRelativeMovement robotRelative, RobotTransform transform, AprilTags aprilTags,
            Camera frontCamera, Camera backCamera) {
        this.robotRelative = robotRelative;
        this.transform = transform;
        this.aprilTags = aprilTags;
        this.frontCamera = frontCamera;
        this.backCamera = backCamera;
    }

    public void setDesiredState(Vector2 movementInput, Rotation2d direction) {
        Rotation2d measured = transform.getRotation();

        var movement = movementInput.rotate(measured.minus(Rotation2d.kCCW_90deg).times(-1).getRadians());

        double turn = 0;
        if (direction != null) {
            var error = direction.minus(measured);
            turn = turningPid.calculate(error.getDegrees(), 0);
        }

        robotRelative.setDesiredState(movement, turn);
    }

    public void moveAndOrient(Vector2 move, Stream<Rotation2d> angles) {
        var measured = VikingMath.normalize(transform.getRotation());
        var angleWithLeastError = angles.min(Comparator.comparing(a -> {
            return Math.abs(a.minus(measured).getDegrees());
        }));
        var turn = angleWithLeastError.map(a -> a.minus(measured).getDegrees() * -0.01).orElse((double) 0);

        robotRelative.setDesiredState(move, turn);
    }

    public Optional<Double> alignToAprilTag(Vector2 centerAt, Camera camera, Stream<Rotation2d> angles) {
        var detection = aprilTags.seen().stream().filter(d -> d.camera == camera).findAny();

        if (detection.isEmpty()) {
            return Optional.empty();
        }
        var d = detection.get();

        var detectionCenter = d.centerCameraSpace();
        var error = new Vector2(
                centerAt.x - detectionCenter.x,
                detectionCenter.y - centerAt.y);
        var multiplyBy = orientPid.calculate(error.getMagnitude(), 0);
        var movement = error.multiply(multiplyBy);

        moveAndOrient(movement, angles);
        return Optional.of(error.getMagnitude());
    }

    public void setForward() {
        transform.setCurrentAs(Rotation2d.kCCW_90deg);
    }

    public Optional<Double> alignToLeftOfReef() {
        return alignToAprilTag(new Vector2(0.7, 0.85), frontCamera, sixties());
    }

    private Stream<Rotation2d> sixties() {
        return IntStream.rangeClosed(0, 5)
                .map(i -> i * 60 + 30)
                .mapToObj(angle -> VikingMath.normalize(Rotation2d.fromDegrees(angle)));
    }

    public Optional<Double> alignToRightOfReef() {
        return alignToAprilTag(new Vector2(0.3, 0.85), frontCamera, sixties());
    }

    public Optional<Double> alignToPickup() {
        var fortyFives = IntStream.iterate(0, i -> i * 90 + 45)
                .mapToObj(angle -> Rotation2d.fromDegrees(angle))
                .limit(4);
        return alignToAprilTag(new Vector2(0.5, 0.1), backCamera, fortyFives);
    }
}
