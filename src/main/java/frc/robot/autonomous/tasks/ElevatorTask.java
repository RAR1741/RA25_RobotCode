package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorTask extends Task {
  private final Elevator m_elevator;
  private final ElevatorState m_targetState;

  public ElevatorTask(ElevatorState state) {
    m_elevator = Elevator.getInstance();
    m_targetState = state;
  }

  @Override
  public void prepare() {
    m_elevator.setState(m_targetState);
    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);
  }

  @Override
  public boolean isFinished() {
    logIsRunning(false);
    return m_elevator.getIsAtState();
  }
}
