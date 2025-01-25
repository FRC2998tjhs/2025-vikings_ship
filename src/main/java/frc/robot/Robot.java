package frc.robot;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  NetworkTable wheelTable = NetworkTableInstance.getDefault().getTable("Swerve");

  AprilTags aprilTags = new AprilTags();

  XboxController controller = new XboxController(0);

  Rotation2d targetAngle = new Rotation2d();

  SwerveGroup swerves = = new SwerveGroup([

  ]);
  SwerveModule frontRight = new SwerveModule(2, 2, 0, new SwerveCalibration(Rotation2d.fromDegrees(-15.), 1.0, true));

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    var vector = new Vector2(controller.getLeftX(), controller.getLeftY());
    if (vector.getMagnitude() > 0.1) {
      targetAngle = new Rotation2d(controller.getRightX(), controller.getRightY());
    } else {

    }
    frontRight.setDesiredState(new SwerveModuleState(controller.getRightTriggerAxis(), targetAngle));
  }
}