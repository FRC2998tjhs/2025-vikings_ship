package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveMovement {
    private Map<SwerveModule, Vector2> moduleOffsets = new HashMap<SwerveModule, Vector2>();

    public SwerveMovement add(SwerveModule module, Vector2 offset) {
        this.moduleOffsets.put(module, offset);
        return this;
    }

    public void setDesiredState(Vector2 direction, double turn) {
        turn = SwerveModule.clamp(turn, -1, 1);
        var ccwTurn = -turn;

        var translationSpeed = direction.getMagnitude();

        var absTurn = Math.abs(ccwTurn);
        var totalSpeed = translationSpeed + absTurn;
        if (totalSpeed < 0.05) {
            stop();
            return;
        }

        for (var entry : moduleOffsets.entrySet()) {
            Vector2 offset = entry.getValue();
            var turnVector = new Vector2(offset.x, offset.y)
                    .rotate((ccwTurn >= 0. ? Rotation2d.kCCW_Pi_2 : Rotation2d.kCW_Pi_2).getRadians())
                    .multiply(absTurn);

            var wheel = turnVector.add(direction);
            if (totalSpeed > 1.) {
                wheel = wheel.divide(totalSpeed);
            }

            var rotation = new Rotation2d(wheel.x, wheel.y);
            entry.getKey().setDesiredState(new SwerveModuleState(wheel.getMagnitude(), rotation));
        }
    }

    public static Rotation2d turnAngle(Vector2 offset, double turnSpeed) {
        var rotate = turnSpeed >= 0. ? Rotation2d.kCCW_Pi_2 : Rotation2d.kCCW_Pi_2;
        return new Rotation2d(offset.x, offset.y).plus(rotate);
    }

    public void stop() {
        for (var module : moduleOffsets.keySet()) {
            module.stop();
        }
    }
}
