package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveToReefTask;
import frc.robot.subsystems.PoseAligner.Branch;

public class TestMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    // queueTask(new DriveTrajectoryTask("please lord i hope"));

    // queueTask(new DriveForwardTask(1.0, 0.1));
    // queueTask(new ElevatorTask(ElevatorState.L3));
    // queueTask(new PrintTask("testing testing"));

    // queueTask(new SequentialTask(
    // new ParallelTask(
    // new EndEffectorTask(EndEffectorState.SCORE_BRANCHES),
    // new WaitTask(1.0)),
    // new EndEffectorTask(EndEffectorState.OFF)));

    // queueTask(new SkippableTask(
    // new ElevatorTask(ElevatorState.L4), 0.1,
    // new ElevatorTask(ElevatorState.STOW)));

    // queueTask(new ArmTask(ArmState.EXTEND));
    // queueTask(new ArmTask(ArmState.STOW));

    // // queueTask(new EndEffectorTask(EndEffectorState.SCORE_BRANCHES));

    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeVariant.LEFT, IntakeState.INTAKE),
    // new IntakeTask(IntakeVariant.RIGHT, IntakeState.INTAKE)));

    // queueTask(new IntakeTask(IntakeVariant.LEFT, IntakeState.STOW));
    // queueTask(new IntakeTask(IntakeVariant.RIGHT, IntakeState.STOW));

    // queueTask(new getACoralOrSomethingIdk);
    // queueTask(new DriveToReefTask(Branch.NONE));
    // queueTask(new DeployThings());
    // queueTask(new DriveToReefTask(Branch.LEFT));
    // queueTask(new Score());

    queueTask(new DriveToReefTask(Branch.NONE));
  }
}
