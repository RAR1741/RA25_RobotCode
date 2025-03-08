package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {
  public OperatorController(int port) {
    super(port, false, false, 0);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput, double triggerActivationThreshold) {
    super(port, useDeadband, useSquaredInput, triggerActivationThreshold);
  }

  int m_leftIntakeButton = Button.LEFT_BUMPER;

  public boolean getWantsLeftIntakeGround() {
    return getRawButton(m_leftIntakeButton);
  }

  public boolean getWantsLeftIntakeStow() {
    return getRawButtonReleased(m_leftIntakeButton);
  }

  int m_rightIntakeButton = Button.RIGHT_BUMPER;

  public boolean getWantsRightIntakeGround() {
    return getRawButton(m_rightIntakeButton);
  }

  public boolean getWantsRightIntakeStow() {
    return getRawButtonReleased(m_rightIntakeButton);
  }

  int m_IntakeEjectButton = Button.B;

  public boolean getWantsIntakeEject() {
    return getRawButton(m_IntakeEjectButton);
  }

  public boolean getWantsIntakeStopEjecting() {
    return getRawButtonReleased(m_IntakeEjectButton);
  }

  public boolean getWantsGoToStow() {
    return this.getRawButtonPressed(Button.A);
  }

  public boolean getWantsGoToL1() {
    return this.getHatPressed(Direction.DOWN);
  }

  public boolean getWantsGoToL2() {
    return this.getHatPressed(Direction.RIGHT);
  }

  public boolean getWantsGoToL3() {
    return this.getHatPressed(Direction.LEFT);
  }

  public boolean getWantsGoToL4() {
    return this.getHatPressed(Direction.UP);
  }

  public boolean getWantsResetElevator() {
    return this.getRawButtonPressed(Button.START);
  }

  int m_scoreButton = Button.X;

  public boolean getWantsScore() {
    return this.getRawButton(m_scoreButton);
  }

  public boolean getWantsEndEffectorOff() {
    return this.getRawButtonReleased(m_scoreButton);
  }

  public boolean getWantsReverseHopper() {
    return this.getRawButtonPressed(Button.Y);
  }

  public boolean getWantsForwardHopper() {
    return this.getRawButtonReleased(Button.Y);
  }
}
