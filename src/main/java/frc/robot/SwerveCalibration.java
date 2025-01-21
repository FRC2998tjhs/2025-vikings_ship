package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveCalibration {
    private Rotation2d wheelMinusForward;

    public SwerveCalibration() {
        this(new Rotation2d());
    }

    public SwerveCalibration(Rotation2d wheelMinusForward) {
        this.wheelMinusForward = wheelMinusForward;
    }

    public SwerveModuleState adjust(SwerveModuleState desiredState, Rotation2d measuredRotation) {
        var result = new SwerveModuleState(desiredState.speedMetersPerSecond, normalize(desiredState.angle));

        measuredRotation = normalize(measuredRotation);

        Rotation2d difference = normalize(measuredRotation.minus(desiredState.angle));

        boolean correctDirection = Math.abs(difference.getDegrees()) <= 90;
        if (!correctDirection) {
            result.speedMetersPerSecond *= -1;
            result.angle = result.angle.plus(Rotation2d.fromDegrees(180));
        }
        result.angle = result.angle.minus(wheelMinusForward);

        return result;
    }

    private Rotation2d normalize(Rotation2d rot) {
        var result = Rotation2d.fromDegrees(rot.getDegrees());

        while (result.getDegrees() > 180) {
            result = result.minus(Rotation2d.fromDegrees(360.));
        }
        while (result.getDegrees() <= -180) {
            result = result.plus(Rotation2d.fromDegrees(360.));
        }

        return result;
    }
}
