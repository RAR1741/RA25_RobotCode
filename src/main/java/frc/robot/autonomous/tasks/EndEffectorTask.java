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
  }

  @Override
  public void update() {
    log(true);
  }

  @Override
  public boolean isFinished() {
    log(false);
    return true;
  }

}