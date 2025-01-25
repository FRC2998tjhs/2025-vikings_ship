package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class SwerveModule {
    private static final double DRIVE_WHILE_TURNING_CORRECTION = 0.16;

    private SwerveCalibration calibration;

    private final TalonFX driveMotor;
    private final SparkMax turningMotor;

    private final PIDController turningPid;

    private final AnalogInput absoluteEncoder;

    public SwerveModule(int driveMotorPort, int turningMotorPort, int absoluteEncoderPort, SwerveCalibration calibration) {
        this.calibration = calibration;
        this.driveMotor = new TalonFX(driveMotorPort);
        this.turningMotor = new SparkMax(turningMotorPort, MotorType.kBrushless);
        this.absoluteEncoder = new AnalogInput(absoluteEncoderPort);

        this.turningPid = new PIDController(.01, 0., 0.);
    }

    private Rotation2d currentRotationReading() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle = (angle * 360.) - 180.;
        return Rotation2d.fromDegrees(angle);
    }

    public void setDesiredState(SwerveModuleState state) {
        var calibrated = calibration.adjust(state, currentRotationReading());

        var rotationSpeed = turningPid.calculate(calibrated.angleError.getDegrees(), 0.);
        rotationSpeed = clamp(rotationSpeed, -1, 1);

        turningMotor.set(rotationSpeed);
        driveMotor.set(calibrated.driveSpeed - DRIVE_WHILE_TURNING_CORRECTION * rotationSpeed);
    }

    private double clamp(double x, double min, double max) {
        if (x < min)
            return min;
        if (x > max)
            return max;
        return x;
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}