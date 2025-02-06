package frc.robot;

import org.dyn4j.geometry.Vector2;

public class FollowAprilTags implements Control {
    private AprilTags aprilTags;
    private SwerveMovement swerves;

    public FollowAprilTags(AprilTags aprilTags, SwerveMovement swerves) {
        this.aprilTags = aprilTags;
        this.swerves = swerves;
    }

    @Override
    public void teleopPeriodic() {
        // TODO(shelbyd): Don't hardcode resolution here.
        var width = 640;
        for (var detection : aprilTags.seen()) {
            var xError = (detection.detection.getCenterX() / width) * 2 - 1;
            swerves.setDesiredState(new Vector2(), xError * 0.2);
        }
    }
}
