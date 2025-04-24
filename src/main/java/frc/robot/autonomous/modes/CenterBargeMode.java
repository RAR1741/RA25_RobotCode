package frc.robot.autonomous.modes;

import java.util.ArrayList;

import frc.robot.autonomous.tasks.ArmTask;
import frc.robot.autonomous.tasks.DoNothingTask;
import frc.robot.autonomous.tasks.DriveTask;
import frc.robot.autonomous.tasks.DriveToPoseTask;
import frc.robot.autonomous.tasks.ElevatorTask;
import frc.robot.autonomous.tasks.EndEffectorTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.SkippableTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.autonomous.tasks.WaitTask.WaitCondition;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.EndEffector.EndEffectorState;
import frc.robot.subsystems.PoseAligner.Barge;
import frc.robot.subsystems.PoseAligner.Branch;

public class CenterBargeMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    ArrayList<Task> tasks = new ArrayList<>();

    // Drive to safe
    tasks.add(new ParallelTask(
        new DriveToPoseTask(Branch.NONE),
        new SequentialTask(
            new WaitTask(WaitCondition.END_EFFECTOR_INDEXED),
            new ParallelTask(
                new ElevatorTask(ElevatorState.L4),
                new ArmTask(ArmState.EXTEND)))));

    // Drive to score
    tasks.add(new DriveToPoseTask(Branch.RIGHT));

    // Score
    tasks.add(new EndEffectorTask(EndEffectorState.SCORE_BRANCHES));

    // Drive to safe pose
    tasks.add(new DriveToPoseTask(Branch.NONE));

    // Dealgae pose
    tasks.add(new EndEffectorTask(EndEffectorState.ALGAE_GRAB));
    tasks.add(new ParallelTask(
        new DriveToPoseTask(Branch.NONE),
        new ArmTask(ArmState.DEALGAE),
        new ElevatorTask(ElevatorState.ALGAE_LOW)));

    // Dealgae
    tasks.add(new SkippableTask(new DriveToPoseTask(Branch.ALGAE), 1.0, new DoNothingTask()));
    tasks.add(new ElevatorTask(ElevatorState.ALGAE_BETWEEN));

    tasks.add(new ParallelTask(
        new DriveToPoseTask(Branch.ALGAE_REVERSE),
        new WaitTask(0.5)));

    queueTasks(tasks);

    // autoScoreAndDealgae(ElevatorState.L4, Branch.RIGHT);
    autoNet(Barge.NEAR);

    queueTask(new DriveTask(5, 0, 2.0));

    queueTasks(getDeAlgaeTasks(ElevatorState.ALGAE_HIGH));
    // autoNet(Barge.NEAR);
  }
}
