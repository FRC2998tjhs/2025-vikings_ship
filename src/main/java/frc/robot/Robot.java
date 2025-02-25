package frc.robot;

import java.util.Arrays;

import org.dyn4j.geometry.Vector2;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  Camera frontCamera = new Camera(1, "Front", Camera.CameraType.Elp);
  // Camera backCamera = new Camera(0, "Back", Camera.CameraType.Elp);

  // Camera frontCamera = backCamera;
  Camera backCamera = frontCamera;

  AprilTags aprilTags = new AprilTags(Arrays.asList(frontCamera, backCamera));

  XboxController controller = new XboxController(0);

  SparkMax dumpMotor = new SparkMax(5, MotorType.kBrushless);

  SwerveModule frontRight = new SwerveModule(
      2, 2, 0,
      new SwerveCalibration(Rotation2d.fromDegrees(8.6), 1.0, false));

  SwerveModule frontLeft = new SwerveModule(
      3, 3, 1,
      new SwerveCalibration(Rotation2d.fromDegrees(42.2), 1.0, false));

  SwerveModule backRight = new SwerveModule(
      1, 1, 2,
      new SwerveCalibration(Rotation2d.fromDegrees(-68.8), 1.0, false));
  SwerveModule backLeft = new SwerveModule(
      4, 4, 3,
      new SwerveCalibration(Rotation2d.fromDegrees(-65.8), -1.0, false));

  RobotRelativeMovement robotRelative = new RobotRelativeMovement()
      .add(frontRight, new Vector2(0.5, 0.6))
      .add(frontLeft, new Vector2(-0.5, 0.6))
      .add(backRight, new Vector2(0.5, -0.6))
      .add(backLeft, new Vector2(-0.5, -0.6));

  Field field = Field.workshop;

  RobotTransform transform = new RobotTransform(new AHRS(NavXComType.kUSB1));

  FieldRelativeMovement fieldRelative = new FieldRelativeMovement(robotRelative, transform, aprilTags, frontCamera,
      backCamera);
  Control control = new Control(controller, fieldRelative, dumpMotor);

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() {
    transform.setCurrentAs(Rotation2d.kCCW_90deg);
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

    control.teleopPeriodic();
  }
}