package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveCalibration {
    private Rotation2d wheelMinusForward;
    private double mpsToMotor;
    private boolean inverseRotation;

    public SwerveCalibration() {
        this(new Rotation2d(), 1.0, false);
    }

    public SwerveCalibration(Rotation2d wheelMinusForward, double mpsToMotor, boolean inverse) {
        this.wheelMinusForward = wheelMinusForward;
        this.mpsToMotor = mpsToMotor;
        this.inverseRotation = inverse;
    }

    public class SwerveTarget {
        public Rotation2d angleError;
        public double driveSpeed;

        private SwerveTarget(double speed, Rotation2d error) {
            this.driveSpeed = speed;
            this.angleError = error;
        }
    }

    public SwerveTarget adjust(SwerveModuleState desiredState, Rotation2d measuredRotation) {
        if (inverseRotation) {
            measuredRotation = measuredRotation.times(-1);
        }
        Rotation2d difference = normalize(desiredState.angle.minus(measuredRotation));

        var result = new SwerveTarget(desiredState.speedMetersPerSecond * mpsToMotor, difference);

        boolean correctDirection = Math.abs(difference.getDegrees()) <= 90;
        if (!correctDirection) {
            result.driveSpeed *= -1;
            result.angleError = result.angleError.minus(Rotation2d.fromDegrees(180));
        }
        result.angleError = result.angleError.plus(wheelMinusForward);
        if (inverseRotation) {
            result.angleError = result.angleError.times(-1);
        }

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
