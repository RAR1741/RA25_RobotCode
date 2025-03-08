package frc.robot.autonomous.modes;

import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PoseAligner.Branch;
import frc.robot.subsystems.PoseAligner.FeederStation;

public class TestMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    // queueTask(new DriveTrajectoryTask("close side"));

    autoScore(ElevatorState.L4, Branch.RIGHT, FeederStation.LEFT);

    // queueTask(new ParallelTask(
    // new DriveToPoseTask(),
    // new CollectCoralTask()));

    autoScore(ElevatorState.L4, Branch.RIGHT, FeederStation.LEFT);
  }
}
