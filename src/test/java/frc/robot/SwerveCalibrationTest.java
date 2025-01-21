package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveCalibrationTest {
    @Test
    void isSameStateWithZeros() {
        var target = new SwerveModuleState(0, new Rotation2d());

        var subject = new SwerveCalibration();
        var newState = subject.adjust(target, new Rotation2d());

        assertEquals(new SwerveModuleState(0, new Rotation2d()), newState);
    }

    @Test
    void negatesSpeedWithOppositeAngle() {
        var target = new SwerveModuleState(1., new Rotation2d());

        var subject = new SwerveCalibration();
        var newState = subject.adjust(target, Rotation2d.fromDegrees(180.));

        assertEquals(-1, newState.speedMetersPerSecond);
    }

    @Test
    void doesNotNegateSpeedWithCorrectAngle() {
        var target = new SwerveModuleState(1., new Rotation2d());

        var subject = new SwerveCalibration();
        var newState = subject.adjust(target, new Rotation2d());

        assertEquals(new SwerveModuleState(1., new Rotation2d()), newState);
    }

    @Test
    void doesNotNegateSpeedWithCloseAngle() {
        var subject = new SwerveCalibration();

        var newState = subject.adjust(new SwerveModuleState(1., new Rotation2d()), Rotation2d.fromDegrees(30.));

        assertEquals(1., newState.speedMetersPerSecond);
    }

    @Test
    void reduces270To90() {
        var subject = new SwerveCalibration();

        var newState = subject.adjust(new SwerveModuleState(1., Rotation2d.fromDegrees(270)), new Rotation2d());

        assertEquals(-1, newState.speedMetersPerSecond);
        assertEquals(90., newState.angle.getDegrees());
    }

    @Test
    void changesNegative270to90() {
        var subject = new SwerveCalibration();

        var newState = subject.adjust(new SwerveModuleState(1., Rotation2d.fromDegrees(-270)), Rotation2d.fromDegrees(-270));

        assertEquals(90., newState.angle.getDegrees());
    }

    @Test
    void needsToAbsDifference() {
        var subject = new SwerveCalibration();

        var newState = subject.adjust(new SwerveModuleState(1., Rotation2d.fromDegrees(90)), new Rotation2d(-45));

        assertEquals(-1., newState.speedMetersPerSecond);
    }

    @Test
    void setsOppositeAngle() {
        var subject = new SwerveCalibration();

        var newState = subject.adjust(new SwerveModuleState(1., Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(170));

        assertEquals(180., newState.angle.getDegrees());
    }

    @Test
    void setsOppositeAngleFromLeftSide() {
        var subject = new SwerveCalibration();

        var newState = subject.adjust(new SwerveModuleState(1., Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(-170));

        assertEquals(180., newState.angle.getDegrees());
    }

    @Test
    void adjustsByOffsetAngle() {
        var subject = new SwerveCalibration(Rotation2d.fromDegrees(30));

        var newState = subject.adjust(new SwerveModuleState(1., Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(90));

        System.out.println(newState.angle.getDegrees());
        assertEquals(-30, newState.angle.getDegrees(), 0.1);
    }
}
