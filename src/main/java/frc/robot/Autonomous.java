package frc.robot;

import org.dyn4j.geometry.Rotation;
import org.dyn4j.geometry.Vector2;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autonomous {

    private FieldRelativeMovement fieldRelative;
    private SparkMax dumpMotor;
    private RobotRelativeMovement robotRelative;

    public Autonomous(FieldRelativeMovement fieldRelative, SparkMax dumpMotor, RobotRelativeMovement robotRelative) {
        this.fieldRelative = fieldRelative;
        this.dumpMotor = dumpMotor;
        this.robotRelative = robotRelative;
    }

    public Command scoreFromCenter() {
        fieldRelative.stop();

        var getToReef = Commands.idle()
                .until(() -> {
                    var error = fieldRelative.alignTo(fieldRelative.leftOfReef, new Vector2(0, -Tunable.autoMoveSpeed));
                    return error.map(e -> e < Tunable.atReefError).orElse(false);
                })
                .withDeadline(Commands.waitSeconds(5))
                .finallyDo(() -> fieldRelative.stop());

        var dump = Commands.run(() -> dumpMotor.set(0.3))
                .withDeadline(Commands.waitSeconds(1))
                .finallyDo(() -> dumpMotor.set(0));

        var dance = Commands.run(() -> fieldRelative.turnRelative(new Vector2(0, Tunable.autoMoveSpeed), 0.3))
                .withDeadline(Commands.waitSeconds(2))
                .andThen(() -> fieldRelative.setDesiredState(new Vector2(), Rotation2d.kCCW_90deg))
                .finallyDo(() -> fieldRelative.stop());
        
        var faceForward = Commands.run(() -> fieldRelative.setDesiredState(new Vector2(), Rotation2d.kCCW_90deg)).withDeadline(Commands.waitSeconds(3));

        return Commands.sequence(
            getToReef
            , Commands.waitSeconds(0.2)
            , dump
            , Commands.waitSeconds(1)
            , dance,
            faceForward
        );
    }

    public Command moveForward() {
        return Commands.run(() -> robotRelative.setDesiredState(new Vector2(0, Tunable.autoMoveSpeed), 0))
                .withDeadline(Commands.waitSeconds(2.0)).finallyDo(() -> robotRelative.stop());
    }
}
