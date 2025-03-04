package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class RobotRelativeMovement {
    private Map<SwerveModule, Vector2> moduleOffsets = new HashMap<SwerveModule, Vector2>();

    public RobotRelativeMovement add(SwerveModule module, Vector2 offset) {
        this.moduleOffsets.put(module, offset);
        return this;
    }

    public void setDesiredState(Vector2 direction, double turn) {
        turn = VikingMath.clamp(turn, -1, 1);
        var ccwTurn = -turn;

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
}
