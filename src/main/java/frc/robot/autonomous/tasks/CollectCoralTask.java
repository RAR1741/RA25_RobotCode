package frc.robot.autonomous.tasks;

import frc.robot.LaserCanHandler;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.intakes.Intake.IntakeState;
import frc.robot.subsystems.intakes.Intakes;
import frc.robot.subsystems.intakes.Intakes.IntakeVariant;

public class CollectCoralTask extends Task {
  private final Intakes m_intakes;
  private final Hopper m_hopper;
  private final LaserCanHandler m_laserCans;
  private final EndEffector m_endEffector;

  public CollectCoralTask() {
    m_intakes = Intakes.getInstance();
    m_hopper = Hopper.getInstance();
    m_endEffector = EndEffector.getInstance();
    m_laserCans = LaserCanHandler.getInstance();
  }

  @Override
  public void prepare() {
    m_intakes.setIntakeState(IntakeVariant.LEFT, IntakeState.INTAKE);
    m_intakes.setIntakeState(IntakeVariant.RIGHT, IntakeState.INTAKE);
    m_hopper.on();
  }

  @Override
  public void update() {
    logIsRunning(true);
  }

  @Override
  public void done() {
    logIsRunning(false);

    m_intakes.setIntakeState(IntakeVariant.LEFT, IntakeState.STOW);
    m_intakes.setIntakeState(IntakeVariant.RIGHT, IntakeState.STOW);
  }

  @Override
  public boolean isFinished() {
    return m_laserCans.getEntranceSeesCoral();
    // return m_endEffector.isSafeToScore();
  }
}
