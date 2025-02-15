package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {
  public OperatorController(int port) {
    super(port, false, false, 0);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput, double triggerActivationThreshold) {
    super(port, useDeadband, useSquaredInput, triggerActivationThreshold);
  }

  public boolean getWantsLeftIntakeGround() {
    return getRawButtonPressed(Button.LEFT_BUMPER);
  }

  public boolean getWantsLeftIntakeStow() {
    return getRawButtonReleased(Button.LEFT_BUMPER);
  }

  public boolean getWantsRightIntakeStow() {
    return getRawButtonReleased(Button.RIGHT_BUMPER);
  }

  public boolean getWantsRightIntakeGround() {
    return getRawButtonPressed(Button.RIGHT_BUMPER);
  }

  public boolean getWantsIntakeEject() {
    return getRawButton(Button.B);
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

  public boolean getWantsArmScore() {
    return this.getHatPressed(Direction.UP);
  }

  public boolean getWantsArmStow() {
    return this.getHatPressed(Direction.DOWN);
  }

  public boolean getWantsScore() {
    return this.getRawButton(Button.X);
  }
}
