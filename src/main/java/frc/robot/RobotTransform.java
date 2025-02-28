package frc.robot;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotTransform {
    private AHRS gyro;
    private Rotation2d gyroOffset = new Rotation2d();

    public RobotTransform(AHRS gyro) {
        this.gyro = gyro;
    }

    public Rotation2d getRotation() {
        var robotRotation = gyro.getRotation2d().minus(gyroOffset);
        return robotRotation;
    }

    public void setCurrentAs(Rotation2d rotation) {
        this.gyroOffset = gyro.getRotation2d().minus(rotation);
    }

    public void resetGyro() {
        this.gyro.reset();
    }
}
