package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.dyn4j.geometry.Vector2;
import org.junit.jupiter.api.Test;

public class SwerveMovementTest {
    @Test
    void turnsCcw() {
        var angle = SwerveMovement.turnAngle(new Vector2(1, 1), 1);
        assertEquals(angle.getDegrees(), 135, 0.1);
    }

    @Test
    void turnsCwForNegativeTurn() {
        var angle = SwerveMovement.turnAngle(new Vector2(1, 1), -1);
        assertEquals(angle.getDegrees(), -45, 0.1);
    }

    @Test
    void negativeOffsets() {
        var angle = SwerveMovement.turnAngle(new Vector2(-1, -1), 1);
        assertEquals(angle.getDegrees(), -45, 0.1);
    }
}
