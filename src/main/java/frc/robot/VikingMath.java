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

    public static Rotation2d controllerStickAngle(double x, double y, double deadzone) {
        var vec = new Vector2(x, -y);
        if (vec.getMagnitude() < deadzone) {
            return null;
        }
        return new Rotation2d(vec.x, vec.y);
    }

    public static Vector2 controllerStickVector(double x, double y, double deadzone) {
        var vec = new Vector2(x, -y);
        if (vec.getMagnitude() < deadzone) {
            return new Vector2();
        }
        var scaled = vec.multiply(1 / (1 - deadzone));
        return scaled.multiply(Math.pow(scaled.getMagnitude(), Tunable.stickExponent));
    }

    public static Rotation2d normalize(Rotation2d rot) {
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
