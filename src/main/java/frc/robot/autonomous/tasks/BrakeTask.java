package frc.robot.autonomous.tasks;

import frc.robot.subsystems.drivetrain.SwerveDrive;

public class BrakeTask extends Task {
  private SwerveDrive m_swerve;
  private boolean m_brake;

  public BrakeTask(boolean brake) {
    m_brake = brake;
    m_swerve = SwerveDrive.getInstance();
  }

  @Override
  public void prepare() {
    m_swerve.setBrake(m_brake);

    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {
    logIsRunning(false);

    m_swerve.drive(0, 0, 0, false);
  }
}
