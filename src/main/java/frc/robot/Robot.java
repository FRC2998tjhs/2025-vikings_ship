package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  AprilTags aprilTags = new AprilTags();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    for (var tag : aprilTags.seen()) {
      System.out.println(tag);
    }
  }
}
