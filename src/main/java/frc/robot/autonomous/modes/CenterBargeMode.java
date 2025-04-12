package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTask;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PoseAligner.Barge;
import frc.robot.subsystems.PoseAligner.Branch;

public class CenterBargeMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    autoScoreAndDealgae(ElevatorState.L4, Branch.RIGHT);
    autoNet(Barge.NEAR);

    queueTask(new DriveTask(5, 0, 1.5));

    queueTasks(getDeAlgaeTasks(ElevatorState.ALGAE_HIGH));
    autoNet(Barge.NEAR);
  }
}
