package frc.robot;

import java.util.Arrays;

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
      new SwerveCalibration(Rotation2d.fromDegrees(-8), 1.0, true));

  SwerveModule frontLeft = new SwerveModule(
      3, 3, 1,
      new SwerveCalibration(Rotation2d.fromDegrees(-43), 1.0, true));

  SwerveMovement swerves = new SwerveMovement()
      .add(frontRight, new Vector2(0.5, 0.6))
      .add(frontLeft, new Vector2(-0.5, 0.6));

  Rotation2d angle = new Rotation2d();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    var leftStick = new Vector2(controller.getLeftX(), controller.getLeftY());
    if (leftStick.getMagnitude() < 0.2) {
      leftStick = new Vector2();
    }

    var turnSpeed = controller.getRightX();
    if (Math.abs(turnSpeed) < 0.1) {
      turnSpeed = 0;
    }
    swerves.setDesiredState(leftStick, turnSpeed);
  }
}