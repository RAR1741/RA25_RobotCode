package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Hopper;

public class HopperTask extends Task {
  private final Hopper m_hopper;
  private final boolean m_enabled;

  public HopperTask(boolean enabled) {
    m_hopper = Hopper.getInstance();
    m_enabled = enabled;
  }

  @Override
  public void prepare() {
  }

  @Override
  public void update() {
    logIsRunning(true);

    if (m_enabled) {
      m_hopper.on();
    } else {
      m_hopper.off();
    }
  }

  @Override
  public void done() {
    logIsRunning(false);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
