package frc.robot.autonomous;

import frc.robot.RobotTelemetry;
import frc.robot.autonomous.modes.AutoModeBase;
import frc.robot.autonomous.modes.DoNothingMode;
import frc.robot.autonomous.modes.TestMode;
import frc.robot.autonomous.tasks.Task;

public class AutoRunner {
  private static AutoRunner m_autoRunner = null;
  private AutoModeBase m_autoMode;

  public static AutoRunner getInstance() {
    if (m_autoRunner == null) {
      m_autoRunner = new AutoRunner();
    }
    return m_autoRunner;
  }

  public enum AutoMode {
    DO_NOTHING,
    TEST
  }

  public Task getNextTask() {
    return m_autoMode.getNextTask();
  }

  public void setAutoMode(AutoMode mode) {
    switch (mode) {
      case DO_NOTHING:
        m_autoMode = new DoNothingMode();
        break;
      case TEST:
        m_autoMode = new TestMode();
        break;
      default:
        RobotTelemetry.print("Invalid auto mode selected. Defaulting to do nothing.");
        m_autoMode = new DoNothingMode();
        break;
    }

    m_autoMode.queueTasks();
  }
}
