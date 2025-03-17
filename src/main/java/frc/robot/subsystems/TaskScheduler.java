package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.autonomous.tasks.Task;

public class TaskScheduler extends Subsystem {
  private ArrayList<Task> m_tasks;
  private static TaskScheduler m_instance = null;

  private TaskScheduler() {
    super("TaskScheduler");

    m_tasks = new ArrayList<>();
  }

  public static TaskScheduler getInstance() {
    if (m_instance == null) {
      m_instance = new TaskScheduler();
    }
    return m_instance;
  }

  public void scheduleTask(Task task) {
    m_tasks.add(task);
  }

  public void scheduleTasks(ArrayList<Task> tasks) {
    // for (Task task : tasks) {
    //   m_tasks.add(task);
    // }
    for(int i = 0; i < tasks.size(); i++) {
      m_tasks.add(tasks.get(i));
    }
  }

  public void removeCurrentTask() {
    if (m_tasks.size() > 0) {
      m_tasks.remove(0);
    }
  }

  public void reset() {
    for (Task task : m_tasks) {
      task.done();
    }
    m_tasks.clear();
  }

  @Override
  public void periodic() {
    // Get the current task
    Task currentTask;
    if (!m_tasks.isEmpty()) {
      currentTask = m_tasks.get(0);
    } else {
      return;
    }

    if (currentTask != null) {
      // Prepare the current task
      if (!currentTask.isPrepared()) {
        currentTask.prepare();
      }

      // Run the current task
      currentTask.update();
      currentTask.updateSim();

      // get rid of the task, if finished
      if (currentTask.isFinished()) {
        currentTask.done();
        removeCurrentTask();
      }
    }
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void stop() {
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @AutoLogOutput(key = "TaskScheduler/NumOfTasks")
  public int getNumberOfTasks() {
    return m_tasks.size();
  }

  @AutoLogOutput(key = "TaskScheduler/hasAnyTasks")
  public boolean hasAnyTasks() {
    return !m_tasks.isEmpty();
  }
}
