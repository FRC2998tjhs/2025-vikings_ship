package frc.robot;

import java.util.List;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveGroup {
    private List<SwerveModule> swerves;

    public SwerveGroup(List<SwerveModule> swerves) {
        this.swerves = swerves;
    }

    public void setDesiredState(SwerveModuleState desired) {
        for (SwerveModule swerveModule : swerves) {
            swerveModule.setDesiredState(desired);
        }
    }
}
