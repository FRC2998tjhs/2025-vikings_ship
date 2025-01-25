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

  SwerveGroup swerves = new SwerveGroup(Arrays.asList(frontRight, frontLeft));

  Rotation2d angle = new Rotation2d();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    var vector = new Vector2(controller.getLeftX(), controller.getLeftY());
    if (vector.getMagnitude() > 0.2) {
      angle = new Rotation2d(vector.x, vector.y);
      swerves.setDesiredState(new SwerveModuleState(vector.getMagnitude(), angle));
    } else {
      swerves.setDesiredState(new SwerveModuleState(0., angle));
    }

    // frontRight.setDesiredState(new SwerveModuleState());
    // frontLeft.setDesiredState(new SwerveModuleState());

    // System.out.println("frontRight: "+ frontRight.currentRotationReading());
    // System.out.println("frontLeft: "+ frontLeft.currentRotationReading());
  }
}