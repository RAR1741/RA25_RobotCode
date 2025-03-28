package frc.robot.autonomous.tasks;

import frc.robot.RobotTelemetry;

public class ParallelTask extends Task {
  private Task[] m_tasks;
  private boolean[] m_finished;

  public ParallelTask(Task... tasks) {
    m_tasks = tasks;
    m_finished = new boolean[tasks.length];
  }

  @Override
  public void prepare() {
    for (Task task : m_tasks) {
      task.prepare();
    }
    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);
    for (int i = 0; i < m_tasks.length; i++) {
      if (!m_finished[i]) {
        m_tasks[i].update();
        if (m_tasks[i].isFinished()) {
          m_tasks[i].done();
          m_finished[i] = true;
        }
      }
    }
  }

  @Override
  public void updateSim() {
    for (Task task : m_tasks) {
      if (!task.isFinished()) {
        task.updateSim();
      }
    }
  }

  @Override
  public boolean isFinished() {
    for (Task task : m_tasks) {
      if (!task.isFinished()) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void done() {
    logIsRunning(false);
    for (Task task : m_tasks) {
      task.done();
    }
    RobotTelemetry.print("Parallel task done");
  }

  public Task[] getTasks() {
    return m_tasks;
  }
}
