package frc.robot.autonomous.tasks;

import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorState;

public class EndEffectorTask extends Task {
  private final EndEffector m_endEffector;
  private final EndEffectorState m_targetState;

  public EndEffectorTask(EndEffectorState state) {
    m_endEffector = EndEffector.getInstance();
    m_targetState = state;
  }

  @Override
  public void prepare() {
    m_endEffector.setState(m_targetState);
    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);
  }

  @Override
  public boolean isFinished() {
    if (m_targetState == EndEffectorState.SCORE_BRANCHES || m_targetState == EndEffectorState.SCORE_TROUGH) {
      return m_endEffector.getEndEffectorState() == EndEffectorState.OFF;
    }
    return true;
  }

  @Override
  public void done() {
    logIsRunning(false);
  }

}
