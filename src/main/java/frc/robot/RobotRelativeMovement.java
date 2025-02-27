package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

public class RobotRelativeMovement {
    private Map<SwerveModule, Vector2> moduleOffsets = new HashMap<SwerveModule, Vector2>();

    private Velocity velocity = new Velocity();

    public RobotRelativeMovement add(SwerveModule module, Vector2 offset) {
        this.moduleOffsets.put(module, offset);
        return this;
    }

    public void setDesiredState(Vector2 direction, double turn) {
        turn = VikingMath.clamp(turn, -1, 1);
        var ccwTurn = -turn;

        this.velocity = velocity.accelerate(direction, turn);
        direction = velocity.direction;
        turn = velocity.turn;

        var translationSpeed = direction.getMagnitude();

        var absTurn = Math.abs(ccwTurn);
        var totalSpeed = translationSpeed + absTurn;

        for (var entry : moduleOffsets.entrySet()) {
            Vector2 offset = entry.getValue();
            var turnVector = new Vector2(offset.x, offset.y)
                    .rotate((ccwTurn >= 0. ? Rotation2d.kCCW_Pi_2 : Rotation2d.kCW_Pi_2).getRadians())
                    .multiply(absTurn);

            var wheel = turnVector.add(direction);
            if (totalSpeed > 1.) {
                wheel = wheel.divide(totalSpeed);
            }

            var rotation = VikingMath.vecAngle(wheel);
            entry.getKey().setDesiredState(new SwerveModuleState(wheel.getMagnitude(), rotation));
        }
    }

    public void stop() {
        for (var module : moduleOffsets.keySet()) {
            module.stop();
        }
    }

    private class Velocity {
        private Vector2 direction;
        private double turn;
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

        private Velocity accelerate(Vector2 targetDir, double targetTurn) {
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
