package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {
  public OperatorController(int port) {
    super(port, false, false, 0);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput, double triggerActivationThreshold) {
    super(port, useDeadband, useSquaredInput, triggerActivationThreshold);
  }

  public boolean getWantsGoToStow() {
    return this.getRawButtonPressed(Button.A);
  }

  public boolean getWantsGoToL1() {
    return this.getRawButtonPressed(Button.Y);
  }

  public boolean getWantsGoToL2() {
    return this.getRawButtonPressed(Button.B);
  }

  public boolean getWantsGoToL3() {
    return this.getRawButtonPressed(Button.LEFT_BUMPER);
  }

  public boolean getWantsGoToL4() {
    return this.getRawButtonPressed(Button.RIGHT_BUMPER);
  }

  public int getWantsScore() {
    if (this.getRawAxis(Axis.RIGHT_TRIGGER) > this.getRawAxis(Axis.LEFT_TRIGGER)) {
      return 1;
    } else if (this.getRawAxis(Axis.RIGHT_TRIGGER) < this.getRawAxis(Axis.LEFT_TRIGGER)) {
      return -1;
    } else {
      return 0;
    }
  }
}
