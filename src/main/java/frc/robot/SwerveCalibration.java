package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveCalibration {
    private Rotation2d wheelMinusForward;
    private double mpsToMotor;
    public boolean invert;

    public SwerveCalibration() {
        this(new Rotation2d(), 1.0, false);
    }

    public SwerveCalibration(Rotation2d wheelMinusForward, double mpsToMotor, boolean invert) {
        this.wheelMinusForward = wheelMinusForward;
        this.mpsToMotor = mpsToMotor;
        this.invert = invert;
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
        Rotation2d actualAngle = measuredRotation.minus(wheelMinusForward);

        Rotation2d difference = VikingMath.normalize(desiredState.angle.minus(actualAngle));

        var result = new SwerveTarget(desiredState.speedMetersPerSecond * mpsToMotor, difference);

        boolean correctDirection = Math.abs(difference.getDegrees()) <= 90;
        if (!correctDirection) {
            result.driveSpeed *= -1;
            result.angleError = result.angleError.minus(Rotation2d.fromDegrees(180));
        }

        return result;
    }
}
