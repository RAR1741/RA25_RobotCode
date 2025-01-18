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
}
