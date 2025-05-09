package frc.robot.autonomous.tasks;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Timer;

public class SkippableTask extends Task {
  private Task m_initialTask;
  private Task m_nextTask;
  private double m_allottedTime;
  private double m_runTime;
  private double m_startTime;
  private boolean m_isSkipped = false;

  public SkippableTask(Task initialTask, double time, Task nextTask) {
    m_initialTask = initialTask;
    m_allottedTime = time;
    m_nextTask = nextTask;
  }

  @Override
  public void prepare() {
    m_startTime = Timer.getFPGATimestamp();
    m_prepared = true;

    m_initialTask.prepare();
  }

  @Override
  public void update() {
    logIsRunning(true);
    m_runTime = Timer.getFPGATimestamp() - m_startTime;

    if (!m_isSkipped) {
      if (!m_initialTask.isFinished()) {
        // Update auto task if the start task has not finished
        m_initialTask.update();

        // Skip the task if it exceeded the allotted time, then prepare the task to run
        if (m_runTime >= m_allottedTime) {
          m_isSkipped = true;

          m_nextTask.prepare();
        }
      }
    } else {
      m_nextTask.update();
    }

    if(m_initialTask.isFinished() && !m_isSkipped) {
      m_initialTask.done();
    }

    if(m_nextTask.isFinished() && m_isSkipped) {
      m_nextTask.done();
    }
  }

  @Override
  public void done() {
    logIsRunning(false);
  }

  @Override
  public boolean isFinished() {
    // Check if the task is skipped
    if (!m_isSkipped) {
      // If the initial task is not skipped and is finished, return true
      return m_initialTask.isFinished();
    } else {
      // If the initial task is skipped and the next task is finished, return true
      return m_nextTask.isFinished();
    }
  }

  @AutoLogOutput
  public double getSkippableTaskRunTime() {
    return m_runTime;
  }
}
