package frc.robot;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  AprilTags aprilTags = new AprilTags();

  XboxController controller = new XboxController(0);

  SwerveModule frontRight = new SwerveModule(
      2, 2, 0,
      new SwerveCalibration(Rotation2d.fromDegrees(8), 1.0, false));

  SwerveModule frontLeft = new SwerveModule(
      3, 3, 1,
      new SwerveCalibration(Rotation2d.fromDegrees(41.8), 1.0, false));

  SwerveModule backRight = new SwerveModule(
      1, 1, 2,
      new SwerveCalibration(Rotation2d.fromDegrees(-68.2), 1.0, true));
  SwerveModule backLeft = new SwerveModule(
      4, 4, 3,
      new SwerveCalibration(Rotation2d.fromDegrees(-65.9), -1.0, true));

  SwerveMovement swerves = new SwerveMovement()
      .add(frontRight, new Vector2(0.5, 0.6))
      .add(frontLeft, new Vector2(-0.5, 0.6))
      .add(backRight, new Vector2(0.5, -0.6))
      .add(backLeft, new Vector2(-0.5, -0.6));

  Rotation2d angle = new Rotation2d();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    // System.out.println();
    // frontRight.calibrate("Front Right");
    // frontLeft.calibrate("Front Left");
    // backRight.calibrate("Back Right");
    // backLeft.calibrate("Back Left");
    // if (true) {
    // return;
    // }

    swerves.stop();

    var leftStick = new Vector2(controller.getLeftX(), -controller.getLeftY());
    if (leftStick.getMagnitude() < 0.2) {
      leftStick = new Vector2();
    }
    leftStick = leftStick.multiply(0.4);
    var angle = new Rotation2d(leftStick.x, leftStick.y);
    var state = new SwerveModuleState(leftStick.getMagnitude(), angle);

    if (controller.getAButton()) {
      frontRight.setDesiredState(state);
    }
    if (controller.getBButton()) {
      frontLeft.setDesiredState(state);
    }
    if (controller.getYButton()) {
      backLeft.setDesiredState(state);
    }
    if (controller.getXButton()) {
      backRight.setDesiredState(state);
    }

    var turnSpeed = controller.getRightX();
    if (Math.abs(turnSpeed) < 0.1) {
      turnSpeed = 0;
    }
    turnSpeed *= 0.4;
    if (true || controller.getRightBumperButton()) {
      swerves.setDesiredState(leftStick, turnSpeed);
    }
  }
}