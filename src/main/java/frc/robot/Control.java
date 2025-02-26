package frc.robot;

import org.dyn4j.geometry.Vector2;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldRelativeMovement.Alignment;

public class Control {
    private XboxController controller;
    private FieldRelativeMovement fieldRelative;

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
            dumpMotor.set(Tunable.dumpSpeed);
        } else if (controller.getLeftBumperButton()) {
            dumpMotor.set(-Tunable.dumpSpeed);
        } else {
            dumpMotor.set(0);
        }

        if (controller.getXButton()) {
            alignToReef(fieldRelative.leftOfReef);
            return;
        }
        if (controller.getBButton()) {
            alignToReef(fieldRelative.rightOfReef);
            return;
        }
        if (controller.getAButton()) {
            fieldRelative.alignTo(fieldRelative.pickup, movement()).isPresent();
            return;
        }

        controllerMovement();
    }

    private void alignToReef(Alignment alignment) {
        var error = fieldRelative.alignTo(alignment, movement());
        if (error.isPresent() && error.get() < Tunable.atReefError) {
            dumpMotor.set(Tunable.dumpSpeed);
        } else {
            dumpMotor.set(-Tunable.dumpSpeed);
        }
    }

    private void controllerMovement() {
        var angle = VikingMath.controllerStickAngle(controller.getRightX(), controller.getRightY(),
                Tunable.rightStickAngleDeadzone);

        fieldRelative.setDesiredState(movement(), angle);
    }

    private Vector2 movement() {
        var movementInput = VikingMath.controllerStickVector(controller.getLeftX(), controller.getLeftY(),
                Tunable.leftStickDeadzone);
        return movementInput.multiply(Tunable.controlMaxSpeed);
    }
}
