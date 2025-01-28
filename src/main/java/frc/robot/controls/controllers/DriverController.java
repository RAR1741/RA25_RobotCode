package frc.robot.controls.controllers;

public class DriverController extends FilteredController {
  public DriverController(int port) {
    super(port, false, false, 0);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput, double triggerActivationThreshold) {
    super(port, useDeadband, useSquaredInput, triggerActivationThreshold);
  }

  // Drive
  public double getForwardAxis() {
    return -this.getFilteredAxis(Axis.LEFT_Y_AXIS);
  }

  public double getStrafeAxis() {
    return -this.getFilteredAxis(Axis.LEFT_X_AXIS);
  }

  public double getTurnAxis() {
    return -this.getFilteredAxis(Axis.RIGHT_X_AXIS);
  }

  public double getSlowScaler() {
    return this.getFilteredAxis(Axis.RIGHT_TRIGGER);
  }

  public double getBoostScaler() {
    return this.getFilteredAxis(Axis.LEFT_TRIGGER);
  }

  // Manual system test modes //
  public double testPositive() {
    return this.getFilteredAxis(Axis.LEFT_TRIGGER);
  }

  public double testNegative() {
    return this.getFilteredAxis(Axis.RIGHT_TRIGGER);
  }

  public boolean getWantsAutoPosition() {
    return this.getRawButton(Button.X);
  }

  public boolean getWantsAutoPositionPressed() {
    return this.getRawButtonPressed(Button.X);
  }
}
