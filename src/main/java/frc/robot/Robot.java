package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import org.dyn4j.geometry.Vector2;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  Camera frontCamera = new Camera(0, Camera.CameraType.LifeCam, new Transform3d());

  AprilTags aprilTags = new AprilTags(Arrays.asList(frontCamera));

  XboxController controller = new XboxController(0);

  SparkMax dumpMotor = new SparkMax(5, MotorType.kBrushless);

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

  Field field = Field.workshop;

  RobotTransform transform = new RobotTransform(new AHRS(NavXComType.kUSB1), aprilTags, field);

  Control control = new XboxControl(controller, swerves);
  // Control control = new FollowAprilTags(aprilTags, swerves);

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
    transform.transform();

    control.teleopPeriodic();

    var speed = controller.getRightTriggerAxis() + -controller.getLeftTriggerAxis();
    dumpMotor.set(speed * 0.5);
  }
}