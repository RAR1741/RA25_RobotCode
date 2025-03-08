package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.CollectCoralTask;
import frc.robot.autonomous.tasks.DriveToPoseTask;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PoseAligner.Branch;
import frc.robot.subsystems.PoseAligner.FeederStation;

public class TestMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    // queueTask(new DriveTrajectoryTask("close side"));

    autoScore(ElevatorState.L4, Branch.LEFT);

    queueTask(new DriveToPoseTask(FeederStation.RIGHT));

    queueTask(new CollectCoralTask());

    autoScore(ElevatorState.L4, Branch.RIGHT);
  }
}
