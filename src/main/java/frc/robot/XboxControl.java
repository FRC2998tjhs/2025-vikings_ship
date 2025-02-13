package frc.robot;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.wpilibj.XboxController;

public class XboxControl implements Control {

    private XboxController controller;
    private SwerveMovement swerves;

    private double maxSpeed = 1.0;
    private double deadzone = 0.1;

    public XboxControl(XboxController controller, SwerveMovement swerves) {
        this.controller = controller;
        this.swerves = swerves;
    }

    @Override
    public void teleopPeriodic() {
        var leftStick = new Vector2(controller.getLeftX(), -controller.getLeftY());
        if (leftStick.getMagnitude() < deadzone) {
            leftStick = new Vector2();
        }
        leftStick = leftStick.multiply(maxSpeed);

        var turnSpeed = controller.getRightX();
        if (Math.abs(turnSpeed) < deadzone) {
            turnSpeed = 0;
        }
        turnSpeed *= maxSpeed;

        swerves.setDesiredState(leftStick, turnSpeed);
    }
}
