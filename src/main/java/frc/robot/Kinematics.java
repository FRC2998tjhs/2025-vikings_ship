package frc.robot;

import org.dyn4j.geometry.Vector2;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;

public class Kinematics {
    private AHRS gyro;
    private Rotation2d gyroOffset = new Rotation2d();

    public Kinematics(AHRS gyro) {
        this.gyro = gyro;
    }

    public Rotation2d rotation() {
        var robotRotation = gyro.getRotation2d().minus(gyroOffset);
        return robotRotation;
    }

    public void setCurrentAs(Rotation2d rotation) {
        this.gyroOffset = gyro.getRotation2d().minus(rotation);
    }

    public void resetGyro() {
        this.gyro.reset();
    }

    public class Velocity {
        public Vector2 direction;
        public double turn;
        private double timestamp;

        private Velocity(Vector2 direction, double turn, double timestamp) {
            this.direction = direction;
            this.turn = turn;
            this.timestamp = timestamp;
        }

        public Velocity() {
            this(new Vector2(), 0., now());
        }

        private static double now() {
            return ((double) RobotController.getFPGATime()) / 1000000.;
        }

        public Velocity accelerate(Vector2 targetDir, double targetTurn) {
            var now = now();

            var dt = now - this.timestamp;

            var dv = targetDir.subtract(this.direction);
            var acceleratedDir = this.direction.add(VikingMath.clampVec(dv, Tunable.movementAcceleration * dt));

            var dTurn = targetTurn - this.turn;
            var turnVelocity = Tunable.turningAcceleration * dt;
            var acceleratedTurn = this.turn + VikingMath.clamp(dTurn, -turnVelocity, turnVelocity);

            return new Velocity(acceleratedDir, acceleratedTurn, now);
        }
    }
}
