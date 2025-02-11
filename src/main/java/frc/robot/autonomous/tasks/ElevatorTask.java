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
    m_elevator.goToElevatorPosition(m_targetState);
  }

  @Override
  public void update() {
    log(true);
  }

  @Override
  public boolean isFinished() {
    return m_elevator.getIsAtState();
  }
}
