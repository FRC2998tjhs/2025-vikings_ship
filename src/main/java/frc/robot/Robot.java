package frc.robot;

import java.util.Arrays;

import org.dyn4j.geometry.Vector2;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

  Kinematics transform = new Kinematics(new AHRS(NavXComType.kUSB1));

  FieldRelativeMovement fieldRelative = new FieldRelativeMovement(robotRelative, transform, aprilTags, frontCamera,
      backCamera);

  int solenoidModuleCan = 7;
  PneumaticsModuleType pneumaticsType = PneumaticsModuleType.CTREPCM;
  Lifting lifting = new Lifting(
    new Solenoid(solenoidModuleCan, pneumaticsType, 1),
    new Solenoid(solenoidModuleCan, pneumaticsType, 0),
    new DoubleSolenoid(solenoidModuleCan, pneumaticsType, 4, 5),
    new Solenoid(solenoidModuleCan, pneumaticsType, 6),
    new Solenoid(solenoidModuleCan, pneumaticsType, 3)
  );

  Control control = new Control(controller, fieldRelative, dumpMotor, lifting, transform);

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() {
    fieldRelative.setBackward();
    lifting.start();
  }

  @Override
  public void autonomousInit() {
    fieldRelative.setBackward();

    new Autonomous(fieldRelative, dumpMotor, robotRelative)
      .scoreFromCenter()
      // .moveForward()
      .schedule();
  }

  @Override
  public void teleopInit() {
    lifting.start();
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