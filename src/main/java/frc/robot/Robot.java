package frc.robot;

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

  SwerveModule swerve = new SwerveModule(3, 4, 0);

  XboxController controller = new XboxController(0);

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    var angle = new Rotation2d(controller.getRightX(), controller.getRightY());
    swerve.setDesiredState(new SwerveModuleState(controller.getRightTriggerAxis(), angle));
  }
}