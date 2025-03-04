package frc.robot;

import org.dyn4j.geometry.Vector2;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.FieldRelativeMovement.Alignment;

public class Control {
    private XboxController controller;
    private FieldRelativeMovement fieldRelative;
    private SparkMax dumpMotor;
    private Lifting lifting;
        private Kinematics kinematics;
    
        public Control(XboxController controller, FieldRelativeMovement fieldRelative, SparkMax dumpMotor,
                Lifting lifting, Kinematics kinematics) {
            this.controller = controller;
            this.fieldRelative = fieldRelative;
            this.dumpMotor = dumpMotor;
            this.lifting = lifting;
            this.kinematics = kinematics;
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

        var dpadDown = controller.getPOV() == 180;
        if (dpadDown) {
            lifting.drop();
        }
        var dpadUp = controller.getPOV() == 0;
        if (dpadUp) {
            lifting.liftRobot();
        }
        var dpadRight = controller.getPOV() == 90;
        if (dpadRight) {
            lifting.coralGrab();
        }
        var dpadLeft = controller.getPOV() == 270;
        if (dpadLeft) {
            lifting.prepForReset();
        }

        if (controller.getRightStickButtonPressed()) {
            lifting.right.set(true);
        }
        if (controller.getRightStickButtonReleased()) {
            lifting.right.set(false);
        }

        if (controller.getLeftStickButtonPressed()) {
            lifting.front.set(Value.kForward);
        }
        if (controller.getLeftStickButtonReleased()) {
            lifting.front.set(Value.kReverse);
        }

        if (controller.getBackButton()) {
            kinematics.resetGyro();
            return;
        }

        double turnFixInput = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
        double turnFix = VikingMath.deadzoneExponent(turnFixInput, Tunable.triggerDeadzone, Tunable.triggerExponent);
        if (Math.abs(turnFix) > 0.01) {
            fieldRelative.turnRelative(movement(), turnFix);
            return;
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

        dumpMotor.set(-Tunable.dumpSpeed);
        if (error.isPresent()) {
            if (error.get() < Tunable.atReefError) {
                dumpMotor.set(Tunable.dumpSpeed);
            } else {
                System.out.println(error.get());
            }
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
