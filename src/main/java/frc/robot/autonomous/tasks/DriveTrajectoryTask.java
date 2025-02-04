package frc.robot.autonomous.tasks;

import frc.robot.RobotTelemetry;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.RARHolonomicDriveController;

public class DriveTrajectoryTask extends Task {
  private final RARHolonomicDriveController m_driveController;
  private final String m_path;

  public DriveTrajectoryTask(String path) {
    m_driveController = new RARHolonomicDriveController(
        RobotConstants.robotConfig.Auto.k_translationConstants,
        RobotConstants.robotConfig.Auto.k_rotationConstants,
        RobotConstants.robotConfig.Robot.k_period);

    m_path = path;
  }

  @Override
  public void prepare() {
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    boolean finished = true;
    if (finished) {
      RobotTelemetry.print("Drive Trajectory task... complete!");
      return true;
    }
    return false;
  }
}
