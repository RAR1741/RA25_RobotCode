package frc.robot.autonomous.modes;

import java.util.ArrayList;

import frc.robot.autonomous.tasks.ArmTask;
import frc.robot.autonomous.tasks.CollectCoralTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTask;
import frc.robot.autonomous.tasks.DriveToPoseTask;
import frc.robot.autonomous.tasks.ElevatorTask;
import frc.robot.autonomous.tasks.EndEffectorTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.autonomous.tasks.WaitTask.WaitCondition;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.EndEffector.EndEffectorState;
import frc.robot.subsystems.PoseAligner.Branch;

public abstract class AutoModeBase {
  private ArrayList<Task> m_tasks;

  public AutoModeBase() {
    m_tasks = new ArrayList<>();
  }

  public Task getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueTask(Task task) {
    m_tasks.add(task);
  }

  public void queueEnd() {
    queueTask(new DriveForwardTask(0, 0));
  }

  public abstract void queueTasks();

  public void deAlgae() {
    queueTask(new ParallelTask(
      new DriveToPoseTask(Branch.NONE),
      new ArmTask(ArmState.EXTEND),
      new ElevatorTask(ElevatorState.ALGAE_LOW)
    ));
    queueTask(new DriveToPoseTask(Branch.ALGAE));
    queueTask(new ParallelTask(
        new SequentialTask(
            new WaitTask(0.5),
            new DriveToPoseTask(Branch.ALGAE_REVERSE)),
        new ArmTask(ArmState.STOW)
    ));
    queueTask(new ElevatorTask(ElevatorState.STOW));
  }

  public void autoScore(ElevatorState elevatorState, Branch branch, int feederStation) {
    ArmState armTarget;
    if (elevatorState == ElevatorState.L4) {
      armTarget = ArmState.EXTEND;
    } else {
      armTarget = ArmState.STOW;
    }

    queueTask(new ParallelTask(
        new DriveToPoseTask(Branch.NONE),
        new SequentialTask(
            new WaitTask(WaitCondition.END_EFFECTOR_INDEXED), 
            new ParallelTask(
                new ElevatorTask(elevatorState),
                new ArmTask(armTarget)))));

    // Drive to score
    queueTask(new DriveToPoseTask(branch));

    // Score
    queueTask(new ParallelTask(
        new EndEffectorTask(EndEffectorState.SCORE_BRANCHES),
        new WaitTask(0.4)));
    queueTask(new EndEffectorTask(EndEffectorState.OFF));

    queueTask(new ParallelTask(
        new DriveToPoseTask(feederStation),
        new CollectCoralTask(),
        new SequentialTask(
            new WaitTask(0.4),
            new ParallelTask(
                new ElevatorTask(ElevatorState.STOW),
                new ArmTask(ArmState.STOW)))));
  }
}
