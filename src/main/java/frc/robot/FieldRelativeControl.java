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

    private static final double CORAL_WIDTH = 0.3048;
    private XboxController controller;
    private SwerveMovement swerves;
    private RobotTransform transform;

    private PIDController turningPid;
    private Camera frontCamera;
    private Camera backCamera;
    private AprilTags aprilTags;

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
    }

    public void teleopPeriodic() {
        if (controller.getYButton()) {
            transform.setCurrentAs(Rotation2d.kCCW_90deg);
        }

        var sixties = IntStream.rangeClosed(0, 5)
                .map(i -> i * 60 + 30)
                .mapToObj(angle -> VikingMath.normalize(Rotation2d.fromDegrees(angle)));
        if (controller.getXButton()) {
            alignToAprilTag(0.7, frontCamera, sixties);
            return;
        }
        if (controller.getBButton()) {
            alignToAprilTag(0.3, frontCamera, sixties);
            return;
        }
        if (controller.getAButton()) {
            var fortyFives = IntStream.iterate(0, i -> i * 90 + 45)
                    .mapToObj(angle -> Rotation2d.fromDegrees(angle))
                    .limit(4);
            alignToAprilTag(0.5, backCamera, fortyFives);
            return;
        }
        // alignToAprilTag(0, frontCamera);
        // if (true)
        // return;

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

    private void alignToAprilTag(double centerAt, Camera camera, Stream<Rotation2d> angles) {
        var detection = aprilTags.seen().stream().filter(d -> d.camera == camera).findAny();

        var x = detection.map(d -> {
            var cameraX = d.centerXCameraSpace();
            return (cameraX - centerAt) * 2 * maxSpeed * 0.5;
        }).orElse((double) 0);

        var movement = new Vector2(x, movementInput().y);

        var measured = VikingMath.normalize(transform.getRotation());
        var angleWithLeastError = angles.min(Comparator.comparing(a -> {
            System.out.println(a.getDegrees());
            return Math.abs(a.minus(measured).getDegrees());
        }));
        System.out.println("");
        System.out.println(measured.getDegrees());
        System.out.println(angleWithLeastError.map(a -> a.getDegrees()));
        var turn = angleWithLeastError.map(a -> a.minus(measured).getDegrees() * -0.01).orElse((double) 0);

        swerves.setDesiredState(movement, turn);
    }
}
