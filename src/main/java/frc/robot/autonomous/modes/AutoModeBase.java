package frc.robot.autonomous.modes;

import java.util.ArrayList;

import frc.robot.autonomous.tasks.ArmTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveToPoseTask;
import frc.robot.autonomous.tasks.ElevatorTask;
import frc.robot.autonomous.tasks.EndEffectorTask;
import frc.robot.autonomous.tasks.HopperTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.autonomous.tasks.WaitTask.WaitCondition;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.EndEffector.EndEffectorState;
import frc.robot.subsystems.PoseAligner;
import frc.robot.subsystems.PoseAligner.Branch;
import frc.robot.subsystems.intakes.Intake.IntakeState;
import frc.robot.subsystems.intakes.Intakes.IntakeVariant;

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

  public void queueTasks(ArrayList<Task> tasks) {
    for (Task task : tasks) {
      m_tasks.add(task);
    }
  }

  public void queueEnd() {
    queueTask(new DriveForwardTask(0, 0));
  }

  public abstract void queueTasks();

  public static ArrayList<Task> getDeAlgaeTasks() {
    ArrayList<Task> tasks = new ArrayList<>();
    ElevatorState elevatorState = PoseAligner.getInstance().getDeAlgaeElevatorState();

    tasks.add(new ParallelTask(
        new DriveToPoseTask(Branch.NONE),
        new ArmTask(ArmState.DEALGAE),
        new ElevatorTask(elevatorState)));

    tasks.add(new DriveToPoseTask(Branch.ALGAE));
    tasks.add(new WaitTask(0.25));

    tasks.add(new ParallelTask(
        new SequentialTask(
            new DriveToPoseTask(Branch.ALGAE_REVERSE)),
            new ArmTask(ArmState.STOW)));

    tasks.add(new ElevatorTask(ElevatorState.L1));

    return tasks;
  }

  public static ArrayList<Task> getAutoScoreTasks(ElevatorState elevatorState, Branch branch) {
    ArrayList<Task> tasks = new ArrayList<>();
    ArmState armTarget;

    boolean CURRENTLY_USING_SAFE_POSE = false;

    boolean useSafePose;
    DriveToPoseTask firstDriveTask;

    if (elevatorState == ElevatorState.L4) {
      armTarget = ArmState.EXTEND;
      useSafePose = true;
      firstDriveTask = new DriveToPoseTask(Branch.NONE);
    } else {
      armTarget = ArmState.STOW;
      useSafePose = false;

      if (CURRENTLY_USING_SAFE_POSE) {
        firstDriveTask = new DriveToPoseTask(Branch.NONE);
      } else {
        firstDriveTask = new DriveToPoseTask(branch);
      }
    }

    tasks.add(new ParallelTask(
        firstDriveTask,
        new SequentialTask(
            new WaitTask(WaitCondition.END_EFFECTOR_INDEXED),
            new ParallelTask(
                new ElevatorTask(elevatorState),
                new ArmTask(armTarget)))));

    // Drive to score
    if (useSafePose || CURRENTLY_USING_SAFE_POSE) {
      tasks.add(new DriveToPoseTask(branch));
    }

    // Score
    if (elevatorState == ElevatorState.L1) {
      tasks.add(new EndEffectorTask(EndEffectorState.SCORE_TROUGH));
    } else {
      tasks.add(new EndEffectorTask(EndEffectorState.SCORE_BRANCHES));
    }

    return tasks;
  }

  public void autoScore(ElevatorState elevatorState, Branch branch, int feederStation) {
    score(elevatorState, branch, new DriveToPoseTask(feederStation));
    // score(elevatorState, branch, new SkippableTask(new
    // DriveToPoseTask(feederStation), 2.2, new DoNothingTask()));
  }

  public void autoScore(ElevatorState elevatorState, Branch branch) {
    score(elevatorState, branch, new DriveToPoseTask(Branch.NONE));
  }

  private void score(ElevatorState elevatorState, Branch branch, Task finalDriveTask) {
    ArmState armTarget;
    if (elevatorState == ElevatorState.L4) {
      armTarget = ArmState.EXTEND;
    } else {
      armTarget = ArmState.STOW;
    }

    // Drive to safe
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
    queueTask(new EndEffectorTask(EndEffectorState.SCORE_BRANCHES));
    // Drive to feeder station
    queueTask(new ParallelTask(
        finalDriveTask,
        new IntakeTask(IntakeVariant.LEFT, IntakeState.INTAKE),
        new IntakeTask(IntakeVariant.RIGHT, IntakeState.INTAKE),
        new HopperTask(true),
        new SequentialTask(
            new WaitTask(0.4),
            new ParallelTask(
                new ElevatorTask(ElevatorState.STOW),
                new ArmTask(ArmState.STOW)))));

    // Wait for the new coral
    // queueTask(new WaitTask(0.5));

    queueTask(new ParallelTask(
        new IntakeTask(IntakeVariant.LEFT, IntakeState.STOW),
        new IntakeTask(IntakeVariant.RIGHT, IntakeState.STOW),
        new WaitTask(0.5)));
  }
}
