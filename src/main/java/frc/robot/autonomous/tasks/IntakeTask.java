package frc.robot.autonomous.tasks;

import frc.robot.subsystems.intakes.Intake.IntakeState;
import frc.robot.subsystems.intakes.Intakes;
import frc.robot.subsystems.intakes.Intakes.IntakeVariant;

public class IntakeTask extends Task {
  private final Intakes m_intakes;
  private final IntakeVariant m_intakeVariant;
  private final IntakeState m_intakeState;

  public IntakeTask(IntakeVariant intakeVariant, IntakeState intakeState) {
    m_intakes = Intakes.getInstance();
    m_intakeVariant = intakeVariant;
    m_intakeState = intakeState;
  }

  @Override
  public void prepare() {
    m_intakes.setIntakeState(m_intakeVariant, m_intakeState);
  }

  @Override
  public void update() {
    logIsRunning(true);
  }

  @Override
  public void done() {
    logIsRunning(false);
  }

  @Override
  public boolean isFinished() {
    return true;
    // return m_intakes.isAtState(m_intakeVariant);
  }
}
