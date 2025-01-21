package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class SwerveModule {

    private SwerveCalibration calibration;

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPid;

    private final AnalogInput absoluteEncoder;

    public SwerveModule(int driveMotorPort, int turningMotorPort, int absoluteEncoderPort,
            SwerveCalibration calibration) {
        this.calibration = calibration;
        this.driveMotor = new TalonFX(driveMotorPort);
        this.turningMotor = new TalonFX(turningMotorPort);
        this.absoluteEncoder = new AnalogInput(absoluteEncoderPort);

        this.turningPid = new PIDController(0.1, 0.0, 0);
    }

    private Rotation2d currentRotationReading() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle = (angle * 2.0 * Math.PI) - Math.PI;
        return Rotation2d.fromRadians(angle);
    }

    public void setDesiredState(SwerveModuleState state) {
        var calibrated = calibration.adjust(state, currentRotationReading());
        driveMotor.set(calibrated.speedMetersPerSecond / 0.2);

        var rotationSpeed = turningPid.calculate(currentRotationReading().getRadians(), state.angle.getRadians());
        rotationSpeed = clamp(rotationSpeed, -0.4, 0.4);
        turningMotor.set(rotationSpeed);
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