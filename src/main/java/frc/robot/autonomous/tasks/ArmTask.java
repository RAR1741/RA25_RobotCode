package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;

public class ArmTask extends Task {
  private Arm m_arm;
  private final ArmState m_targetState;

  public ArmTask(ArmState state) {
    m_arm = Arm.getInstance();
    m_targetState = state;
  }

  @Override
  public void prepare() {
    m_arm.setArmState(m_targetState);//Go Go Gadget Better Jacob
  }

  @Override
  public void update() {
    //Go Go Gadget Update
    log(true);
  }

  @Override
  public boolean isFinished() {
    log(false);
    return m_arm.getIsAtState();
  }

}