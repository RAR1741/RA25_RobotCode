package frc.robot.autonomous.modes;

import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PoseAligner.Branch;

public class CenterBargeMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    autoScore(ElevatorState.L4, Branch.RIGHT);
  }
}
