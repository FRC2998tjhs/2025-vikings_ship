package frc.robot;

import org.dyn4j.geometry.Vector2;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autonomous {

    private FieldRelativeMovement fieldRelative;
    private SparkMax dumpMotor;

    public Autonomous(FieldRelativeMovement fieldRelative, SparkMax dumpMotor) {
        this.fieldRelative = fieldRelative;
        this.dumpMotor = dumpMotor;
    }

    public Command command() {
        var moveToReef = new Vector2(0, -Tunable.autoMoveSpeed);
        var getToReef = Commands.idle()
                .until(() -> {
                    var error = fieldRelative.alignTo(fieldRelative.leftOfReef, moveToReef);
                    return error.map(e -> e < Tunable.atReefError).orElse(false);
                })
                .withDeadline(Commands.waitSeconds(5))
                .finallyDo(() -> fieldRelative.stop());

        var dump = Commands.run(() -> dumpMotor.set(0.3))
                .withDeadline(Commands.waitSeconds(1))
                .finallyDo(() -> dumpMotor.set(0));

        var dance = Commands.run(() -> fieldRelative.turnRelative(moveToReef.multiply(-1), 0.3))
                .withDeadline(Commands.waitSeconds(2))
                .finallyDo(() -> fieldRelative.stop());

        return Commands.sequence(getToReef, Commands.waitSeconds(0.2), dump, Commands.waitSeconds(1), dance);
    }

}
