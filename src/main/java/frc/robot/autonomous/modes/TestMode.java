package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTask;

public class TestMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    queueTask(new DriveTask(2.5, 0, 5));
    queueTask(new DriveTask(0, 2.5, 5));
    queueTask(new DriveTask(2.5, 0, 5));
    queueTask(new DriveTask(0, 2.5, 5));
    queueTask(new DriveTask(2.5, 0, 5));
  }
}
