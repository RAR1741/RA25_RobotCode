package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.ArmTask;
import frc.robot.autonomous.tasks.DriveToReefTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ElevatorTask;
import frc.robot.autonomous.tasks.EndEffectorTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.EndEffector.EndEffectorState;
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

    // queueTask(new DriveTrajectoryTask("close side"));

    queueTask(new DriveTrajectoryTask("close side"));

    // Go to safe pose
    queueTask(new DriveToReefTask(Branch.NONE));

    // Extend to score
    queueTask(new ParallelTask(
        new ElevatorTask(ElevatorState.L4),
        new ArmTask(ArmState.EXTEND)));

    // Drive to score
    queueTask(new DriveToReefTask(Branch.LEFT));

    // Score
    queueTask(new ParallelTask(
        new EndEffectorTask(EndEffectorState.SCORE_BRANCHES),
        new WaitTask(0.5)));
    queueTask(new EndEffectorTask(EndEffectorState.OFF));

    // Drive back to safe pose
    queueTask(new DriveToReefTask(Branch.NONE));

    // Stow
    queueTask(new ParallelTask(
        new ElevatorTask(ElevatorState.STOW),
        new ArmTask(ArmState.STOW)));
  }
}
