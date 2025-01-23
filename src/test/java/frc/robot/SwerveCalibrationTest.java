package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveCalibrationTest {
    private Rotation2d deg(double degrees) {
        return Rotation2d.fromDegrees(degrees);
    }

    @Test
    void isSameStateWithZeros() {
        var target = new SwerveModuleState(0, deg(0.));

        var result = new SwerveCalibration().adjust(target, deg(0.));

        assertEquals(result.driveSpeed, 0.);
        assertEquals(result.angleError.getDegrees(), 0.);
    }

    @Test
    void negatesSpeedWithOppositeAngle() {
        var target = new SwerveModuleState(1., deg(0.));

        var result = new SwerveCalibration().adjust(target, deg(180.));

        assertEquals(result.driveSpeed, -1.);
        assertEquals(result.angleError.getDegrees(), 0.);
    }

    @Test
    void doesNotNegateSpeedWithCorrectAngle() {
        var target = new SwerveModuleState(1., deg(0.));
        var result = new SwerveCalibration().adjust(target, deg(0.));

        assertEquals(result.driveSpeed, 1.);
        assertEquals(result.angleError.getDegrees(), 0.);
    }

    @Test
    void doesNotNegateSpeedWithCloseAngle() {
        var target = new SwerveModuleState(1., deg(0.));
        var result = new SwerveCalibration().adjust(target, deg(30.));

        assertEquals(result.driveSpeed, 1.);
        assertEquals(result.angleError.getDegrees(), -30., 0.1);
    }

    @Test
    void reduces270To90() {
        var target = new SwerveModuleState(1., deg(271.));
        var result = new SwerveCalibration().adjust(target, deg(0.));

        assertEquals(result.driveSpeed, 1.);
        assertEquals(result.angleError.getDegrees(), -89., 0.1);
    }

    @Test
    void changesNegative270to90() {
        var target = new SwerveModuleState(1., deg(-269.));
        var result = new SwerveCalibration().adjust(target, deg(-270.));

        assertEquals(result.driveSpeed, 1.);
        assertEquals(result.angleError.getDegrees(), 1., 0.1);
    }

    @Test
    void goesToCloserPosition() {
        var target = new SwerveModuleState(1., deg(90));
        var result = new SwerveCalibration().adjust(target, deg(-45));

        assertEquals(result.driveSpeed, -1.);
        assertEquals(result.angleError.getDegrees(), -45., 0.1);
    }

    @Test
    void adjustsByOffsetAngle() {
        var target = new SwerveModuleState(1., deg(0));
        var result = new SwerveCalibration(deg(30.)).adjust(target, deg(90));

        assertEquals(result.driveSpeed, 1.);
        assertEquals(result.angleError.getDegrees(), -60., 0.1);
    }
}
