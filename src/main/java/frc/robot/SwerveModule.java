package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPid;

    private final AnalogInput absoluteEncoder;

    public SwerveModule(int driveMotorPort, int turningMotorPort, int absoluteEncoderPort) {
        this.driveMotor = new TalonFX(driveMotorPort);
        this.turningMotor = new TalonFX(turningMotorPort);
        this.absoluteEncoder = new AnalogInput(absoluteEncoderPort);

        this.turningPid = new PIDController(0.1, 0.1, 0);
    }

    public Rotation2d getRotation() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle = (angle * 2.0 * Math.PI) - Math.PI;
        angle *= -1.;
        return Rotation2d.fromRadians(angle);
    }

    public void setDesiredState(SwerveModuleState state) {
        state.optimize(getRotation());

        driveMotor.set(state.speedMetersPerSecond / 0.2);

        var current = getRotation().getDegrees();
        var target = state.angle.getDegrees();

        System.out.println("Diff:   " + (current - target));
        double speed = turningPid.calculate(getRotation().getRadians(), state.angle.getRadians());

        speed = clamp(speed, -0.4, 0.4);
        turningMotor.set(speed);
    }

    private double clamp(double x, double min, double max) {
        if (x < min) return min;
        if (x > max) return max;
        return x;
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}