package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Comparator;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import org.dyn4j.geometry.Vector2;
import org.dyn4j.geometry.Vector3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class FieldRelativeControl implements Control {

    private XboxController controller;
    private SwerveMovement swerves;
    private RobotTransform transform;

    private Camera frontCamera;
    private Camera backCamera;
    private AprilTags aprilTags;

    private PIDController turningPid;
    private PIDController orientPid;

    private final double maxSpeed = 0.4;

    public FieldRelativeControl(XboxController controller, SwerveMovement swerves, RobotTransform transform,
            Camera frontCamera, Camera backCamera, AprilTags aprilTags) {
        this.controller = controller;
        this.swerves = swerves;
        this.transform = transform;
        this.frontCamera = frontCamera;
        this.backCamera = backCamera;
        this.aprilTags = aprilTags;

        this.turningPid = new PIDController(0.005, 0, 0);
        this.orientPid = new PIDController(1.5, 0.2, 0);
    }

    public void teleopPeriodic() {
        if (controller.getYButton()) {
            transform.setCurrentAs(Rotation2d.kCCW_90deg);
        }

        var sixties = IntStream.rangeClosed(0, 5)
                .map(i -> i * 60 + 30)
                .mapToObj(angle -> VikingMath.normalize(Rotation2d.fromDegrees(angle)));
        if (controller.getXButton()) {
            alignToAprilTag(new Vector2(0.7, 0.85), frontCamera, sixties);
            return;
        }
        if (controller.getBButton()) {
            alignToAprilTag(new Vector2(0.3, 0.85), frontCamera, sixties);
            return;
        }
        if (controller.getAButton()) {
            var fortyFives = IntStream.iterate(0, i -> i * 90 + 45)
                    .mapToObj(angle -> Rotation2d.fromDegrees(angle))
                    .limit(4);
            alignToAprilTag(new Vector2(0.5, 0.1), backCamera, fortyFives);
            return;
        }

        controllerMovement();
    }

    private void controllerMovement() {
        Rotation2d measured = transform.getRotation();

        var movement = movementInput().rotate(measured.minus(Rotation2d.kCCW_90deg).times(-1).getRadians());

        swerves.setDesiredState(movement.multiply(maxSpeed), turn());
    }

    private Vector2 movementInput() {
        return VikingMath.controllerStickVector(controller.getLeftX(), controller.getLeftY(), 0.15);
    }

    private double turn() {
        Rotation2d measured = transform.getRotation();

        var target = VikingMath.controllerStickAngle(controller.getRightX(), controller.getRightY(), 0.8);
        if (target == null) {
            return 0;
        }
        var error = target.minus(measured);
        return turningPid.calculate(error.getDegrees(), 0);
    }

    private void alignToAprilTag(Vector2 centerAt, Camera camera, Stream<Rotation2d> angles) {
        var detection = aprilTags.seen().stream().filter(d -> d.camera == camera).findAny();

        detection.ifPresent(d -> {
            var detectionCenter = d.centerCameraSpace();
            var error = new Vector2(
                    centerAt.x - detectionCenter.x,
                    detectionCenter.y - centerAt.y);
            var multiplyBy = orientPid.calculate(error.getMagnitude(), 0);
            var movement = error.multiply(multiplyBy);

            var measured = VikingMath.normalize(transform.getRotation());
            var angleWithLeastError = angles.min(Comparator.comparing(a -> {
                return Math.abs(a.minus(measured).getDegrees());
            }));
            var turn = angleWithLeastError.map(a -> a.minus(measured).getDegrees() * -0.01).orElse((double) 0);

            swerves.setDesiredState(movement, turn);
        });

        if (detection.isEmpty()) {
            controllerMovement();
        }
    }
}
