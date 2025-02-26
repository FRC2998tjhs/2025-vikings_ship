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
import edu.wpi.first.wpilibj2.command.Commands;

public class Robot extends TimedRobot {
  private static final boolean DO_CALIBRATION = false;

  Camera frontCamera = new Camera(0, "Front", Camera.CameraType.Elp);
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
    fieldRelative.setForward();
  }

  @Override
  public void autonomousInit() {
    fieldRelative.setForward();

    var getToReef = Commands.idle()
        .until(() -> {
          var error = fieldRelative.alignTo(fieldRelative.leftOfReef, new Vector2(0, Tunable.autoMoveSpeed));
          return error.map(e -> e < Tunable.atReefError).orElse(false);
        })
        .withDeadline(Commands.waitSeconds(5))
        .finallyDo(() -> fieldRelative.stop());

    var dump = Commands.run(() -> dumpMotor.set(0.3))
        .withDeadline(Commands.waitSeconds(1))
        .finallyDo(() -> dumpMotor.set(0));

    Commands.sequence(getToReef, Commands.waitSeconds(0.2), dump).schedule();
  }

  @Override
  public void teleopPeriodic() {
    if (DO_CALIBRATION) {
      System.out.println();
      frontRight.calibrate("Front Right");
      frontLeft.calibrate("Front Left");
      backRight.calibrate("Back Right");
      backLeft.calibrate("Back Left");
      return;
    }

    control.teleopPeriodic();
  }
}