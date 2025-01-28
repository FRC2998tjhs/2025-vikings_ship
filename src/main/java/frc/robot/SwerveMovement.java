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
        var rotateOffsets = turn >= 0. ? Rotation2d.fromRadians(Math.PI / 2) : Rotation2d.fromRadians(-Math.PI / 2);
        var absTurn = Math.abs(turn);

        var forTranslation = new Rotation2d(direction.x, direction.y);
        var translationSpeed = direction.getMagnitude();

        var totalWeight = translationSpeed + absTurn;
        if (totalWeight < 0.1) {
            for (var module : moduleOffsets.keySet()) {
                module.setDesiredState(new SwerveModuleState());
            }
            return;
        }

        var speedWeight = translationSpeed / totalWeight;
        var turnWeight = absTurn / totalWeight;

        for (var entry : moduleOffsets.entrySet()) {
            var forTurning = new Rotation2d(entry.getValue().x, entry.getValue().y);
            // .plus(rotateOffsets);

            var rotation = forTurning.times(turnWeight).plus(forTranslation.times(speedWeight));
            var speed = translationSpeed * speedWeight + turn * turnWeight;

            entry.getKey().setDesiredState(new SwerveModuleState(speed, rotation));
        }
    }
}
