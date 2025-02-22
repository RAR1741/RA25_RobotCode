package frc.robot.subsystems;

import java.util.ArrayList;

import frc.robot.RobotTelemetry;
import frc.robot.autonomous.tasks.Task;

public class TaskScheduler extends Subsystem {
  private final ArrayList<Task> m_tasks;
  private static TaskScheduler m_instance = null;

  private TaskScheduler() {
    super("TaskScheduler");

    m_tasks = new ArrayList<>();
  }

  public static TaskScheduler getInstance() {
    if(m_instance == null) {
      m_instance = new TaskScheduler();
    }
    return m_instance;
  }

  public void scheduleTask(Task task) {
    m_tasks.add(task);
  }

  public void clearAllTasks() {
    m_tasks.clear();
  }

  public void skipCurrentTask() {
    if(m_tasks.size() > 0) {
      m_tasks.remove(0);
    }
  }

  @Override
  public void reset() {
    clearAllTasks();
  }

  @Override
  public void periodic() {
    // Get the current task
    Task currentTask;
    if(!m_tasks.isEmpty()) {
      currentTask = m_tasks.get(0);
    } else {
      return;
    }

    if (currentTask != null) {
      // Prepare the current task
      if(!currentTask.isPrepared()) {
        currentTask.prepare();
      }

      // Run the current task
      currentTask.update();
      currentTask.updateSim();

      // get rid of the task, if finished
      if (currentTask.isFinished()) {
        currentTask.done();
        skipCurrentTask();
      }
    }
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

}
