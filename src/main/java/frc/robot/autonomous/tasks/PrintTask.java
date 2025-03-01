package frc.robot.autonomous.tasks;

import frc.robot.RobotTelemetry;

public class PrintTask extends Task {
  private final String m_message;

  public PrintTask(String message) {
    m_message = message;
  }

  @Override
  public void prepare() {
    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);
    RobotTelemetry.print(m_message);
  }

  @Override
  public boolean isFinished() {
    logIsRunning(false);
    return true;
  }

}
