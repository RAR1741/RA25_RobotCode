package frc.robot.autonomous.tasks;

import frc.robot.RobotTelemetry;

public class DoNothingTask extends Task {
  @Override
  public void prepare() {
    RobotTelemetry.print("Starting do nothing auto...");
    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);

    RobotTelemetry.print("Do nothing auto complete");
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {
    logIsRunning(false);
  }
}
