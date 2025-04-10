package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.autonomous.tasks.ArmTask;
import frc.robot.autonomous.tasks.DriveDistanceTask;
import frc.robot.autonomous.tasks.DriveTask;
import frc.robot.autonomous.tasks.DriveToPoseTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ElevatorTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PoseAligner.Branch;
import frc.robot.subsystems.PoseAligner.FeederStation;
import frc.robot.subsystems.intakes.Intake.IntakeState;
import frc.robot.subsystems.intakes.Intakes.IntakeVariant;

public class TestMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    // queueTask(new DriveTrajectoryTask("close side"));

    // autoScore(ElevatorState.L4, Branch.RIGHT, FeederStation.LEFT);

    // autoScore(ElevatorState.L4, Branch.RIGHT, FeederStation.LEFT);

    // deAlgae();
    // queueTasks(getDeAlgaeTasks());

    queueTask(new DriveTrajectoryTask("i am speed"));
    queueTasks(getAutoScoreTasks(ElevatorState.L4, Branch.RIGHT));
    queueTask(new DriveToPoseTask(Branch.NONE));
    queueTask(new ArmTask(ArmState.STOW));
    queueTask(new ElevatorTask(ElevatorState.STOW));
    queueTask(new DriveDistanceTask(new Rotation2d(Units.degreesToRadians(90.0))));
    queueTask(new IntakeTask(IntakeVariant.RIGHT, IntakeState.INTAKE));
    queueTask(new DriveTask(0.0, -0.75, 1.5));
    queueTask(new IntakeTask(IntakeVariant.RIGHT, IntakeState.STOW));

    queueTask(new ParallelTask(
        new IntakeTask(IntakeVariant.RIGHT, IntakeState.STOW),
        new DriveTask(0.0, -0.75, 0.1)));
    
    autoScore(ElevatorState.L4, Branch.LEFT, FeederStation.RIGHT);
  }
}
