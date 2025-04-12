package frc.robot.autonomous.modes;

import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PoseAligner.Barge;
import frc.robot.subsystems.PoseAligner.Branch;

public class CenterBargeMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    autoScoreAndDealgae(ElevatorState.L4, Branch.RIGHT);
    autoNet(Barge.NEAR);
  }
}
