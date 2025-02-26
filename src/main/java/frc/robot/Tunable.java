package frc.robot;

import org.dyn4j.geometry.Vector2;

public class Tunable {
    public static final double maxAlignToAprilTagTurnSpeed = 0.3;
    public static final Vector2 leftOfReefAprilTagInCamera = new Vector2(0.74, 0.853);
    public static final Vector2 rightOfReefAprilTagInCamera = new Vector2(0.314, 0.855);
    public static final Vector2 pickupAprilTagCamera = new Vector2(0.5, 0.5);

    public static final double dumpSpeed = 0.3;

    public static final double autoDumpFor = 1;
    public static final double autoMoveSpeed = 0.2;
    public static final double atReefError = 0.03;

    public static final double leftStickDeadzone = 0.15;
    public static final double rightStickAngleDeadzone = 0.8;

    public static final double controlMaxSpeed = 0.5;
    public static final double stickExponent = 1.2;
}
