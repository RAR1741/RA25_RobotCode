package frc.robot.autonomous.modes;

import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PoseAligner.Branch;
import frc.robot.subsystems.PoseAligner.FeederStation;

public class RightMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    autoScore(ElevatorState.L4, Branch.LEFT, FeederStation.RIGHT);
    autoScore(ElevatorState.L4, Branch.LEFT, FeederStation.RIGHT);
    // autoScore(ElevatorState.L4, Branch.RIGHT, FeederStation.RIGHT);
  }
}
