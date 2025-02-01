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
        var totalWeight = translationSpeed + absTurn;
        if (totalWeight < 0.1) {
            for (var module : moduleOffsets.keySet()) {
                module.setDesiredState(new SwerveModuleState());
            }
            return;
        }

        var forTranslation = new Rotation2d(direction.x, direction.y);

        var speedWeight = translationSpeed / totalWeight;
        var turnWeight = absTurn / totalWeight;

        for (var entry : moduleOffsets.entrySet()) {
            Vector2 offset = entry.getValue();
            var forTurning = turnAngle(offset, ccwTurn);

            var rotation = forTurning.times(turnWeight).plus(forTranslation.times(speedWeight));
            var speed = translationSpeed * speedWeight + ccwTurn * turnWeight;

            var state = new SwerveModuleState(speed, rotation);
            entry.getKey().setDesiredState(state);
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
