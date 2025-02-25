package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;

public class Control {
    private XboxController controller;
    private FieldRelativeMovement fieldRelative;

    private double maxSpeed = 0.5;
    private SparkMax dumpMotor;

    public Control(XboxController controller, FieldRelativeMovement fieldRelative, SparkMax dumpMotor) {
        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.dumpMotor = dumpMotor;
    }

    void teleopPeriodic() {
        if (controller.getStartButton()) {
            fieldRelative.setForward();
        }

        if (controller.getRightBumperButton()) {
            dumpMotor.set(0.3);
        } else if (controller.getLeftBumperButton()) {
            dumpMotor.set(-0.3);
        } else {
            dumpMotor.set(0);
        }

        var aligned = false;
        if (controller.getXButton()) {
            aligned = fieldRelative.alignToLeftOfReef().isPresent();
        }
        if (controller.getBButton()) {
            aligned = fieldRelative.alignToRightOfReef().isPresent();
        }
        if (controller.getAButton()) {
            aligned = fieldRelative.alignToPickup().isPresent();
        }

        if (aligned) {
            return;
        }

        controllerMovement();
    }

    private void controllerMovement() {
        var movement = VikingMath.controllerStickVector(controller.getLeftX(), controller.getLeftY(), 0.15);
        var angle = VikingMath.controllerStickAngle(controller.getRightX(), controller.getRightY(), 0.8);
        fieldRelative.setDesiredState(movement.multiply(maxSpeed), angle);
    }
}
