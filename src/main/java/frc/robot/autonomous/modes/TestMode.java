package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class TestMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    queueTask(new DriveTrajectoryTask("pleasegodihope"));
  }
}
