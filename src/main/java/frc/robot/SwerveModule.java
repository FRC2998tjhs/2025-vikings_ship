package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

    public Rotation2d currentRotationReading() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle = (angle * 360.) - 180.;
        return Rotation2d.fromDegrees(angle);
    }

    public void setDesiredState(SwerveModuleState state) {
        Rotation2d reading = currentRotationReading();
        var calibrated = calibration.adjust(state, reading);

        if (Math.abs(calibrated.driveSpeed) < 0.05) {
            stop();
            return;
        }

        // System.out.println("Read: " + reading.getDegrees());
        // System.out.println("Target: " + state.angle.getDegrees());
        // System.out.println("Error: " + calibrated.angleError.getDegrees());

        var needToMove = calibrated.angleError.getDegrees();
        var rotationSpeed = turningPid.calculate(needToMove, 0.);
        rotationSpeed = clamp(rotationSpeed, -1, 1);
        if (calibration.invert) {
            rotationSpeed *= -1;
        }

        turningMotor.set(rotationSpeed);
        driveMotor.set(calibrated.driveSpeed - DRIVE_WHILE_TURNING_CORRECTION * rotationSpeed);
    }

    public static double clamp(double x, double min, double max) {
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

    public void calibrate(String name) {
        var reading = currentRotationReading();
        if (Math.abs(reading.getDegrees()) > 90.) {
            reading = reading.plus(Rotation2d.k180deg);
        }
        System.out.println("Set " + name + " angle to: " + reading.getDegrees());
    }
}