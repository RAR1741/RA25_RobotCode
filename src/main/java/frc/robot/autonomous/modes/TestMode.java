package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.ArmTask;
import frc.robot.autonomous.tasks.ElevatorTask;
import frc.robot.autonomous.tasks.EndEffectorTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PrintTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.SkippableTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.EndEffector.EndEffectorState;
import frc.robot.subsystems.intakes.Intake.IntakeState;
import frc.robot.subsystems.intakes.Intakes.IntakeVariant;

public class TestMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    // queueTask(new DriveTrajectoryTask("please lord i hope"));

    // queueTask(new DriveForwardTask(1.0, 0.1));
    queueTask(new ElevatorTask(ElevatorState.L3));
    queueTask(new PrintTask("testing testing"));

    queueTask(new SequentialTask(
        new ParallelTask(
            new EndEffectorTask(EndEffectorState.SCORE_BRANCHES),
            new WaitTask(1.0)),
        new EndEffectorTask(EndEffectorState.OFF)));

    queueTask(new SkippableTask(
        new ElevatorTask(ElevatorState.L4), 0.1,
        new ElevatorTask(ElevatorState.STOW)));

    queueTask(new ArmTask(ArmState.EXTEND));
    queueTask(new ArmTask(ArmState.STOW));

    // queueTask(new EndEffectorTask(EndEffectorState.SCORE_BRANCHES));

    queueTask(new ParallelTask(
        new IntakeTask(IntakeVariant.LEFT, IntakeState.INTAKE),
        new IntakeTask(IntakeVariant.RIGHT, IntakeState.INTAKE)));

    queueTask(new IntakeTask(IntakeVariant.LEFT, IntakeState.STOW));
    queueTask(new IntakeTask(IntakeVariant.RIGHT, IntakeState.STOW));
  }
}
