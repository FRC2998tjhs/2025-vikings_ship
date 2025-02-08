package frc.robot;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.geometry.Rotation2d;

public class VikingMath {
    public static double clamp(double x, double min, double max) {
        if (x < min)
            return min;
        if (x > max)
            return max;
        return x;
    }

    public static Rotation2d vecAngle(Vector2 vec) {
        if (vec.getMagnitude() < 0.001) {
            return new Rotation2d();
        } else {
            return new Rotation2d(vec.x, vec.y);
        }
    }
}
