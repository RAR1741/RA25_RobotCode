package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.DriveDistanceTask;

public class TestMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    // queueTask(new DriveTrajectoryTask("close side"));

    // autoScore(ElevatorState.L4, Branch.RIGHT, FeederStation.LEFT);

    // autoScore(ElevatorState.L4, Branch.RIGHT, FeederStation.LEFT);

    // deAlgae();
    // queueTasks(getDeAlgaeTasks());

    queueTask(new DriveDistanceTask(new Rotation2d(45)));
  }
}
