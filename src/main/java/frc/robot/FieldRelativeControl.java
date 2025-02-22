package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class FieldRelativeControl implements Control {

    private XboxController controller;
    private SwerveMovement swerves;
    private RobotTransform transform;

    private PIDController turningPid;

    public FieldRelativeControl(XboxController controller, SwerveMovement swerves, RobotTransform transform) {
        this.controller = controller;
        this.swerves = swerves;
        this.transform = transform;

        this.turningPid = new PIDController(0.005, 0, 0);
    }

    public void teleopPeriodic() {
        if (controller.getYButton()) {
            transform.setCurrentAs(Rotation2d.kCCW_90deg);
        }

        Rotation2d measured = transform.getRotation();
        var target = VikingMath.controllerStickAngle(controller.getRightX(), controller.getRightY(), 0.8);
        target = target == null ? measured : target;
        var error = target.minus(measured);
        double turn = target == null ? 0 : turningPid.calculate(error.getDegrees(), 0);

        var movementInput = VikingMath.controllerStickVector(controller.getLeftX(), controller.getLeftY(), 0.15);
        var movement = movementInput.rotate(measured.minus(Rotation2d.kCCW_90deg).times(-1).getRadians());

        swerves.setDesiredState(movement.multiply(0.2), turn);
    }
}
